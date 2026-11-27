#include "doctest/doctest.h"

#include "test_support.h"
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace {

constexpr float WAVE_AMPLITUDE = 0.1f;
constexpr float WAVE_NUMBER = 5.3f;
constexpr int KNOT_NODES = 6;
constexpr float NO_GLIDE = std::numeric_limits<float>::infinity();
constexpr double AMP_AT_KNOTS[KNOT_NODES] = {0.2, 0.21, 0.23, 0.4, 0.88, 1.0};

struct Pose {
    Vector2 pos;
    float phi;
};

struct Piece {
    float length;
    float curvature;
};

// y down: heading (cos phi, -sin phi), positive curvature turns screen-left.
struct Track {
    Pose start;
    std::vector<Piece> pieces;

    static Pose advance(Pose p, float length, float curvature) {
        if (curvature == 0.0f)
            return {{p.pos.x + std::cos(p.phi) * length,
                     p.pos.y - std::sin(p.phi) * length},
                    p.phi};
        float phi = p.phi + curvature * length;
        return {{p.pos.x + (std::sin(phi) - std::sin(p.phi)) / curvature,
                 p.pos.y + (std::cos(phi) - std::cos(p.phi)) / curvature},
                phi};
    }

    Pose at(float x) const {
        if (x <= 0.0f)
            return advance(start, x, 0.0f);
        Pose p = start;
        for (Piece piece : pieces) {
            if (x <= piece.length)
                return advance(p, x, piece.curvature);
            p = advance(p, piece.length, piece.curvature);
            x -= piece.length;
        }
        return p;
    }
};

struct PairedSwim {
    BodyParams waved_params = bodyParams(WAVE_AMPLITUDE, KNOT_NODES, 0);
    BodyParams plain_params = bodyParams(0.0f, KNOT_NODES, 0);
    const Track &track;
    float v;
    float dt;
    float x;
    float r;
    SharkBody waved;
    SharkBody plain;

    PairedSwim(const Track &t, float x0, float speed, float step,
               float yaw_rate = 0.0f)
        : track(t), v(speed), dt(step), x(x0), r(yaw_rate) {
        Kinematics kin = kinHere();
        waved = seedSharkBody(kin, waved_params);
        plain = seedSharkBody(kin, plain_params);
    }

    Kinematics kinHere() const {
        Pose p = track.at(x);
        return kinAt(p.pos, p.phi, v, r);
    }

    void step() {
        x += v * dt;
        Kinematics kin = kinHere();
        waved = updateSharkBody(waved, kin, waved_params, dt);
        plain = updateSharkBody(plain, kin, plain_params, dt);
    }

    void swimTo(float target_x) {
        while (x < target_x)
            step();
    }
};

Vector2 swimmingDirection(const std::vector<Vector2> &spine, int i) {
    int last = (int)spine.size() - 1;
    int ahead = i == 0 ? 0 : i - 1;
    int behind = i == last ? last : i + 1;
    Vector2 d = {spine[ahead].x - spine[behind].x,
                 spine[ahead].y - spine[behind].y};
    float n = std::hypot(d.x, d.y);
    return {d.x / n, d.y / n};
}

// outside_is_left: +1 when the outside of the turn is screen-left, -1 when
// it is screen-right.
void checkOffsets(const SharkBody &waved, const SharkBody &plain, double a,
                  double outside_is_left, double wave_scale = 1.0) {
    REQUIRE(waved.spine.size() == KNOT_NODES);
    REQUIRE(plain.spine.size() == KNOT_NODES);
    const double AL = WAVE_AMPLITUDE * L;
    for (int i = 0; i < KNOT_NODES; ++i) {
        CAPTURE(i);
        double s = (double)i / (KNOT_NODES - 1);
        double theta = WAVE_NUMBER * s - waved.phase;
        double h = AL * AMP_AT_KNOTS[i] * std::sin(theta);
        double asymmetry = a * AL * AMP_AT_KNOTS[i] *
                           (2.0 / PI_D - 4.0 / (3.0 * PI_D) * std::cos(2.0 * theta));
        double expected_left = wave_scale * (h + outside_is_left * asymmetry);

        Vector2 fwd = swimmingDirection(plain.spine, i);
        Vector2 left = {fwd.y, -fwd.x};
        Vector2 d = {waved.spine[i].x - plain.spine[i].x,
                     waved.spine[i].y - plain.spine[i].y};
        double along_left = (double)d.x * left.x + (double)d.y * left.y;
        double measured_left = std::copysign(std::hypot(d.x, d.y), along_left);
        CHECK(std::abs(measured_left - expected_left) <= 1e-3);
    }
}

struct YawSegment {
    float duration;
    float r;
};

using Swim = std::vector<YawSegment>;

std::vector<std::vector<Vector2>> spinesOfSwim(const Swim &swim, float v,
                                               const BodyParams &params,
                                               float dt) {
    Kinematics kin = kinAt({14.0f, 9.0f}, 0.9f, v);
    SharkBody body = seedSharkBody(kin, params);
    std::vector<std::vector<Vector2>> spines;
    for (YawSegment segment : swim) {
        int frames = (int)std::lround(segment.duration / dt);
        for (int f = 0; f < frames; ++f) {
            kin = turnStep(kin, segment.r, dt);
            body = updateSharkBody(body, kin, params, dt);
            spines.push_back(body.spine);
        }
    }
    return spines;
}

std::vector<std::vector<Vector2>> spinesOfCommandedSwim(
    const Swim &swim, float v, const BodyParams &params, float dt) {
    Kinematics kin = kinAt({14.0f, 9.0f}, 0.9f, v);
    SharkBody body = seedSharkBody(kin, params);
    std::vector<std::vector<Vector2>> spines;
    for (YawSegment segment : swim) {
        int frames = (int)std::lround(segment.duration / dt);
        for (int f = 0; f < frames; ++f) {
            kin = integrate(kin, {v, segment.r}, SHARK_PARAMS, dt);
            body = updateSharkBody(body, kin, params, dt);
            spines.push_back(body.spine);
        }
    }
    return spines;
}

double secondDifference(const std::vector<std::vector<Vector2>> &spines,
                        size_t frame, int node) {
    Vector2 before = spines[frame - 1][node];
    Vector2 now = spines[frame][node];
    Vector2 after = spines[frame + 1][node];
    return std::hypot((double)after.x - 2.0 * now.x + before.x,
                      (double)after.y - 2.0 * now.y + before.y);
}

} // namespace

TEST_SUITE("turn asymmetry") {

TEST_CASE("steady circles: each node is offset by h plus a A L amp "
          "(2/pi - 4/(3 pi) cos 2 theta) toward the outside") {
    struct Expected {
        float curvature;
        double a;
    };
    const float v = 1.0f, dt = 1.0f / 60.0f;
    for (Expected e : {Expected{0.0f, 0.0}, Expected{0.4f, 0.25},
                       Expected{0.8f, 0.5}, Expected{1.6f, 0.5}}) {
        for (float direction : {1.0f, -1.0f}) {
            CAPTURE(e.curvature);
            CAPTURE(direction);
            const Track circle{{{12.0f, 9.0f}, 0.6f},
                               {{4.0f * L, direction * e.curvature}}};
            PairedSwim swim(circle, 0.0f, v, dt);
            swim.swimTo(2.0f * L);
            for (int sample = 0; sample < 5; ++sample) {
                CAPTURE(swim.waved.phase);
                checkOffsets(swim.waved, swim.plain, e.a, -direction);
                for (int k = 0; k < 7; ++k)
                    swim.step();
            }
        }
    }
}

TEST_CASE("straight at cruise with uneven dt: the tail tip's left and right "
          "peaks over a beat are equal within 2 %") {
    const BodyParams params = bodyParams(WAVE_AMPLITUDE);
    const float uneven[] = {1.0f / 30.0f, 0.007f, 1.0f / 144.0f, 0.019f,
                            0.001f, 1.0f / 41.0f};
    const float v = 1.0f;
    const float beat_period = 2.0f / v;
    Kinematics kin = kinAt({2.0f, 3.0f}, 2.4f, v);
    const Vector2 origin = kin.pos;
    const Vector2 left = {kin.heading.y, -kin.heading.x};
    SharkBody body = seedSharkBody(kin, params);
    double peak_left = 0.0, peak_right = 0.0;
    float t = 0.0f;
    for (int step = 0; t < L / v + beat_period; ++step) {
        float dt = uneven[step % 6];
        t += dt;
        kin = straightStep(kin, dt);
        body = updateSharkBody(body, kin, params, dt);
        if (t < L / v)
            continue;
        Vector2 tip = body.spine.back();
        double lateral_left = (double)(tip.x - origin.x) * left.x +
                              (double)(tip.y - origin.y) * left.y;
        peak_left = std::fmax(peak_left, lateral_left);
        peak_right = std::fmax(peak_right, -lateral_left);
    }
    CHECK(peak_left > 0.2);
    CHECK(std::abs(peak_left / peak_right - 1.0) <= 0.02);
}

TEST_CASE("straight, 90 degree arc, straight: a follows the body turn as the "
          "body passes through the arc, and is 0 more than L past it") {
    const float radius = 1.25f, v = 1.0f, dt = 1.0f / 60.0f;
    const float curvature = 1.0f / radius;
    const float arc_length = (float)(PI_D / 2.0) * radius;
    const double a_max = 0.5, full_curvature = 0.8;
    auto geometricA = [&](float x) {
        double in_arc = std::fmax(
            0.0, std::fmin((double)x, arc_length) - std::fmax(x - L, 0.0f));
        double body_turn = curvature * in_arc / L;
        return a_max * std::fmin(body_turn / full_curvature, 1.0);
    };
    for (float direction : {1.0f, -1.0f}) {
        for (float phase0 : {0.0f, 1.3f, 2.6f, 4.2f}) {
            CAPTURE(direction);
            CAPTURE(phase0);
            const Track track{{{4.0f, 9.0f}, -0.3f},
                              {{arc_length, direction * curvature},
                               {2.0f * L, 0.0f}}};
            PairedSwim swim(track, -L - 0.5f, v, dt);
            swim.waved.phase = phase0;
            for (float target : {1.0f, arc_length + 1.0f, arc_length + 2.5f,
                                 arc_length + L + 0.5f}) {
                CAPTURE(target);
                swim.swimTo(target);
                double a = geometricA(swim.x);
                CAPTURE(a);
                if (target > arc_length + L)
                    CHECK(a == 0.0);
                checkOffsets(swim.waved, swim.plain, a, -direction);
            }
        }
    }
}

TEST_CASE("scripted swims at 60 fps: asymmetry adds at most 2 mm to any node's "
          "second difference in time") {
    const float dt = 1.0f / 60.0f;
    const float r_max = 0.8f, v_cruise = 1.0f, v_min = 0.5f;
    const float half_turn_s = (float)PI_D / r_max;
    const float quarter_turn_s = half_turn_s / 2.0f;
    struct Scripted {
        std::string name;
        float v;
        Swim yaw;
    };
    for (float direction : {1.0f, -1.0f}) {
        const float r = direction * r_max;
        for (const Scripted &scripted : {
                 Scripted{"cruise half turn", v_cruise,
                          {{L / v_cruise, 0.0f},
                           {half_turn_s, r},
                           {2.0f * L / v_cruise, 0.0f}}},
                 Scripted{"tightest half turn", v_min,
                          {{L / v_min, 0.0f},
                           {half_turn_s, r},
                           {2.0f * L / v_min, 0.0f}}},
                 Scripted{"cruise S-turn", v_cruise,
                          {{L / v_cruise, 0.0f},
                           {quarter_turn_s, r},
                           {quarter_turn_s, -r},
                           {2.0f * L / v_cruise, 0.0f}}}}) {
            CAPTURE(direction);
            CAPTURE(scripted.name);
            BodyParams with = bodyParams(WAVE_AMPLITUDE);
            with.glide_full_yaw_rate = NO_GLIDE;
            BodyParams without = with;
            without.asymmetry_max = 0.0f;
            const auto asymmetric = spinesOfSwim(scripted.yaw, scripted.v, with, dt);
            const auto symmetric =
                spinesOfSwim(scripted.yaw, scripted.v, without, dt);
            REQUIRE(asymmetric.size() == symmetric.size());
            double worst_excess = -1.0;
            for (size_t frame = 1; frame + 1 < asymmetric.size(); ++frame)
                for (int node = 0; node < NODES; ++node)
                    worst_excess = std::fmax(
                        worst_excess, secondDifference(asymmetric, frame, node) -
                                          secondDifference(symmetric, frame, node));
            CHECK(worst_excess <= 2e-3);
        }
    }
}

}

TEST_SUITE("turn glide") {

TEST_CASE("at half the full-glide yaw rate the whole wave, asymmetry included, "
          "is halved") {
    const float v = 1.0f, dt = 1.0f / 60.0f;
    const float curvature = 0.4f;
    const double a = 0.25;
    const double half_glide_scale = 0.5;
    for (float direction : {1.0f, -1.0f}) {
        CAPTURE(direction);
        const Track circle{{{12.0f, 9.0f}, 0.6f},
                           {{4.0f * L, direction * curvature}}};
        PairedSwim swim(circle, 0.0f, v, dt, direction * v * curvature);
        swim.swimTo(2.0f * L);
        for (int sample = 0; sample < 5; ++sample) {
            CAPTURE(swim.waved.phase);
            checkOffsets(swim.waved, swim.plain, a, -direction,
                         half_glide_scale);
            for (int k = 0; k < 7; ++k)
                swim.step();
        }
    }
}

TEST_CASE("full-rate tightest turn: no node sways more than 1 mm over a beat") {
    const float v_min = 0.5f, r_max = 0.8f, dt = 1.0f / 60.0f;
    const float beat_period = 2.0f / v_min;
    const BodyParams waved_params = bodyParams(WAVE_AMPLITUDE);
    const BodyParams plain_params = bodyParams(0.0f);
    for (float r : {r_max, -r_max}) {
        CAPTURE(r);
        Kinematics kin = kinAt({15.0f, 8.0f}, 0.3f, v_min, r);
        SharkBody waved = seedSharkBody(kin, waved_params);
        SharkBody plain = seedSharkBody(kin, plain_params);
        std::vector<double> lowest(NODES, 1e9), highest(NODES, -1e9);
        for (float t = 0.0f; t < 2.0f * L / v_min + beat_period; t += dt) {
            kin = turnStep(kin, r, dt);
            waved = updateSharkBody(waved, kin, waved_params, dt);
            plain = updateSharkBody(plain, kin, plain_params, dt);
            if (t < 2.0f * L / v_min)
                continue;
            for (int i = 0; i < NODES; ++i) {
                Vector2 fwd = swimmingDirection(plain.spine, i);
                Vector2 left = {fwd.y, -fwd.x};
                double lateral =
                    (double)(waved.spine[i].x - plain.spine[i].x) * left.x +
                    (double)(waved.spine[i].y - plain.spine[i].y) * left.y;
                lowest[i] = std::fmin(lowest[i], lateral);
                highest[i] = std::fmax(highest[i], lateral);
            }
        }
        for (int i = 0; i < NODES; ++i) {
            CAPTURE(i);
            CHECK(highest[i] - lowest[i] <= 1e-3);
        }
    }
}

TEST_CASE("commanded swims at 60 fps: the glide adds at most 2 mm to any node's "
          "second difference in time") {
    const float dt = 1.0f / 60.0f;
    const float r_max = 0.8f, v_cruise = 1.0f, v_min = 0.5f;
    const float half_turn_s = (float)PI_D / r_max;
    const float quarter_turn_s = half_turn_s / 2.0f;
    struct Commanded {
        std::string name;
        float v;
        Swim yaw;
    };
    for (float direction : {1.0f, -1.0f}) {
        const float r = direction * r_max;
        for (const Commanded &commanded : {
                 Commanded{"cruise half turn", v_cruise,
                           {{L / v_cruise, 0.0f},
                            {half_turn_s, r},
                            {2.0f * L / v_cruise, 0.0f}}},
                 Commanded{"tightest half turn", v_min,
                           {{L / v_min, 0.0f},
                            {half_turn_s, r},
                            {2.0f * L / v_min, 0.0f}}},
                 Commanded{"cruise S-turn", v_cruise,
                           {{L / v_cruise, 0.0f},
                            {quarter_turn_s, r},
                            {quarter_turn_s, -r},
                            {2.0f * L / v_cruise, 0.0f}}}}) {
            CAPTURE(direction);
            CAPTURE(commanded.name);
            BodyParams gliding = bodyParams(WAVE_AMPLITUDE);
            BodyParams beating = gliding;
            beating.glide_full_yaw_rate = NO_GLIDE;
            const auto glide =
                spinesOfCommandedSwim(commanded.yaw, commanded.v, gliding, dt);
            const auto beat =
                spinesOfCommandedSwim(commanded.yaw, commanded.v, beating, dt);
            REQUIRE(glide.size() == beat.size());
            double worst_excess = -1.0;
            for (size_t frame = 1; frame + 1 < glide.size(); ++frame)
                for (int node = 0; node < NODES; ++node)
                    worst_excess = std::fmax(
                        worst_excess, secondDifference(glide, frame, node) -
                                          secondDifference(beat, frame, node));
            CHECK(worst_excess <= 2e-3);
        }
    }
}

TEST_CASE("after a sudden full-rate turn the wave scale falls as exp(-t / tau), "
          "with uneven dt") {
    const BodyParams params = bodyParams(WAVE_AMPLITUDE);
    const double tau = 1.0;
    const float uneven[] = {1.0f / 30.0f, 0.004f, 1.0f / 144.0f, 0.021f,
                            0.0005f, 1.0f / 33.0f, 0.0f, 0.012f};
    Kinematics kin = kinAt({15.0f, 8.0f}, 0.3f, 1.0f);
    SharkBody body = seedSharkBody(kin, params);
    REQUIRE(body.wave_scale == 1.0f);
    double t = 0.0;
    for (int step = 0; t < 3.0; ++step) {
        CAPTURE(t);
        float dt = uneven[step % 8];
        t += dt;
        kin = turnStep(kin, 0.8f, dt);
        body = updateSharkBody(body, kin, params, dt);
        CHECK(std::abs(body.wave_scale - std::exp(-t / tau)) <= 1e-4);
    }
}

}
