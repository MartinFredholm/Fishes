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

void checkOffsets(const SharkBody &waved, const SharkBody &plain,
                  double wave_scale) {
    REQUIRE(waved.spine.size() == KNOT_NODES);
    REQUIRE(plain.spine.size() == KNOT_NODES);
    const double AL = WAVE_AMPLITUDE * L;
    for (int i = 0; i < KNOT_NODES; ++i) {
        CAPTURE(i);
        double s = (double)i / (KNOT_NODES - 1);
        double h = AL * AMP_AT_KNOTS[i] * std::sin(WAVE_NUMBER * s - waved.phase);
        double expected_left = wave_scale * h;

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

TEST_SUITE("turn glide") {

TEST_CASE("at half the full-glide yaw rate the whole wave is halved") {
    const float v = 1.0f, dt = 1.0f / 60.0f;
    const float curvature = 0.4f;
    const double half_glide_scale = 0.5;
    for (float direction : {1.0f, -1.0f}) {
        CAPTURE(direction);
        const Track circle{{{12.0f, 9.0f}, 0.6f},
                           {{4.0f * L, direction * curvature}}};
        PairedSwim swim(circle, 0.0f, v, dt, direction * v * curvature);
        swim.swimTo(2.0f * L);
        for (int sample = 0; sample < 5; ++sample) {
            CAPTURE(swim.waved.phase);
            checkOffsets(swim.waved, swim.plain, half_glide_scale);
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
