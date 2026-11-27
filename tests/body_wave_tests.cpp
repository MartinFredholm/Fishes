#include "doctest/doctest.h"

#include "test_support.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace {

constexpr float WAVE_AMPLITUDE = 0.1f;

struct Frame {
    Vector2 origin;
    Vector2 fwd;
    float along(Vector2 p) const {
        return (p.x - origin.x) * fwd.x + (p.y - origin.y) * fwd.y;
    }
    float across(Vector2 p) const {
        return (p.x - origin.x) * fwd.y - (p.y - origin.y) * fwd.x;
    }
};

void checkSegmentLengths(const SharkBody &body) {
    const float rest = L / (NODES - 1);
    REQUIRE(body.spine.size() == NODES);
    for (int i = 0; i + 1 < NODES; ++i) {
        CAPTURE(i);
        CHECK(std::abs(dist(body.spine[i], body.spine[i + 1]) - rest) <=
              0.001f * rest);
    }
}

double angleDiff(double a, double b) { return std::remainder(a - b, 2.0 * PI_D); }

double lateral(Vector2 p, Vector2 origin, Vector2 fwd) {
    return (double)(p.x - origin.x) * fwd.y - (double)(p.y - origin.y) * fwd.x;
}

float worstBend(const std::vector<Vector2> &spine) {
    float worst = 0.0f;
    for (size_t i = 1; i + 1 < spine.size(); ++i) {
        Vector2 u = {spine[i].x - spine[i - 1].x, spine[i].y - spine[i - 1].y};
        Vector2 w = {spine[i + 1].x - spine[i].x, spine[i + 1].y - spine[i].y};
        worst = std::max(worst, std::abs(std::atan2(u.x * w.y - u.y * w.x,
                                                    u.x * w.x + u.y * w.y)));
    }
    return worst;
}

std::vector<double> lateralOffsets(float phase, int nodes) {
    const BodyParams params = bodyParams(WAVE_AMPLITUDE, nodes, 0);
    Kinematics kin = kinAt({3.0f, 1.0f}, 0.0f, 0.0f);
    SharkBody body = seedSharkBody(kin, params);
    body.phase = phase;
    SharkBody next = updateSharkBody(body, kin, params, 0.0f);
    std::vector<double> h(nodes);
    for (int i = 0; i < nodes; ++i)
        h[i] = lateral(next.spine[i], kin.pos, kin.heading);
    return h;
}

} // namespace

TEST_SUITE("body wave") {

TEST_CASE("one update advances the phase by pi v dt") {
    const BodyParams params = bodyParams(WAVE_AMPLITUDE);
    const float u = 1.7f;
    const float dt = 0.02f;
    Kinematics kin = kinAt({5.0f, 5.0f}, 0.3f, u);
    SharkBody body = seedSharkBody(kin, params);
    body.phase = 1.0f;

    SharkBody next = updateSharkBody(body, straightStep(kin, dt), params, dt);

    double expected = 1.0 + PI_D * u * dt;
    CHECK(std::abs(angleDiff(next.phase, expected)) <= 1e-6);
}

TEST_CASE("wave offset at the blog's six amplitude points") {
    const double amp[] = {0.2, 0.21, 0.23, 0.4, 0.88, 1.0};
    const BodyParams params = bodyParams(WAVE_AMPLITUDE, 6, 0);
    const float phi = 2.1f;
    const float u = 1.0f;
    const float dt = 1.0f / 60.0f;

    for (float phase0 : {0.4f, 0.4f + (float)PI_D / 2.0f}) {
        CAPTURE(phase0);
        Kinematics kin = kinAt({10.0f, 8.0f}, phi, u);
        SharkBody body = seedSharkBody(kin, params);
        body.phase = phase0;
        Kinematics moved = straightStep(kin, dt);
        SharkBody next = updateSharkBody(body, moved, params, dt);
        REQUIRE(next.spine.size() == 6);

        Frame frame{moved.pos, moved.heading};
        double phase = phase0 + PI_D * u * dt;
        double h_tail = 0.1 * L * amp[5] * std::sin(5.3 - phase);
        float side = frame.across(next.spine[5]) * h_tail > 0 ? 1.0f : -1.0f;
        for (int i = 0; i < 6; ++i) {
            CAPTURE(i);
            double s = i / 5.0;
            double h = 0.1 * L * amp[i] * std::sin(5.3 * s - phase);
            CHECK(std::abs(side * frame.across(next.spine[i]) - h) < 1e-4);
            CHECK(std::abs(-frame.along(next.spine[i]) - s * L) < 1e-4);
        }
    }
}

TEST_CASE("amplitude envelope passes through the six points and has no slope "
          "jump at them") {
    const int nodes = 301;
    const double knots[] = {0.2, 0.21, 0.23, 0.4, 0.88, 1.0};
    std::vector<double> h_sin = lateralOffsets(0.7f, nodes);
    std::vector<double> h_cos = lateralOffsets(0.7f + (float)PI_D / 2.0f, nodes);
    std::vector<double> amp(nodes);
    for (int i = 0; i < nodes; ++i)
        amp[i] = std::hypot(h_sin[i], h_cos[i]) / (0.1 * L);

    const double ds = 1.0 / (nodes - 1);
    for (int k = 0; k < 6; ++k) {
        CAPTURE(k);
        int i = k * 60;
        CHECK(amp[i] == doctest::Approx(knots[k]).epsilon(1e-3));
        if (k == 0 || k == 5)
            continue;
        double left = (amp[i] - amp[i - 1]) / ds;
        double right = (amp[i + 1] - amp[i]) / ds;
        CHECK(std::abs(right - left) < 0.3);
    }
}

TEST_CASE("every segment within 0.1 % of L/23 after every update") {
    const BodyParams params = bodyParams(WAVE_AMPLITUDE);
    const float dt = 1.0f / 60.0f;

    SUBCASE("straight") {
        Kinematics kin = kinAt({5.0f, 5.0f}, 0.3f, 1.0f);
        SharkBody body = seedSharkBody(kin, params);
        for (int step = 0; step < 600; ++step) {
            CAPTURE(step);
            kin = straightStep(kin, dt);
            body = updateSharkBody(body, kin, params, dt);
            checkSegmentLengths(body);
        }
    }
    SUBCASE("turning") {
        for (float v : {0.5f, 1.0f, 2.5f}) {
            CAPTURE(v);
            const float r = 0.8f;
            Kinematics kin = kinAt({15.0f, 8.0f}, 0.0f, v);
            SharkBody body = seedSharkBody(kin, params);
            for (int step = 0; step < 600; ++step) {
                CAPTURE(step);
                kin = turnStep(kin, step < 300 ? r : -r, dt);
                body = updateSharkBody(body, kin, params, dt);
                checkSegmentLengths(body);
            }
        }
    }
}

TEST_CASE("slow, tightest turn: the wave adds at most 0.3 rad to the path's "
          "bend") {
    const BodyParams params = bodyParams(WAVE_AMPLITUDE);
    const float v_min = 0.5f, r_max = 0.8f, dt = 1.0f / 60.0f;
    const float path_bend_rad = L / (NODES - 1) / TIGHTEST_TURN_RADIUS;
    const float wave_bend_allowance_rad = 0.3f;
    for (float r : {r_max, -r_max}) {
        CAPTURE(r);
        Kinematics kin = kinAt({15.0f, 8.0f}, 0.3f, v_min, r);
        SharkBody body = seedSharkBody(kin, params);
        for (int step = 0; step < 600; ++step) {
            CAPTURE(step);
            kin = turnStep(kin, r, dt);
            body = updateSharkBody(body, kin, params, dt);
            CHECK(worstBend(body.spine) <=
                  path_bend_rad + wave_bend_allowance_rad);
        }
    }
}

TEST_CASE("straight at cruise, peak tail-tip offset over a beat is 0.30 m "
          "+/- 20 %") {
    const BodyParams params = bodyParams(WAVE_AMPLITUDE);
    const float u = 1.0f;
    const float dt = 1.0f / 60.0f;
    const float beat_period = 2.0f / u;
    Kinematics kin = kinAt({3.0f, 8.0f}, -0.4f, u);
    const Frame frame{kin.pos, kin.heading};
    SharkBody body = seedSharkBody(kin, params);

    float t = 0.0f;
    for (; t < L / u; t += dt) {
        kin = straightStep(kin, dt);
        body = updateSharkBody(body, kin, params, dt);
    }
    float peak = 0.0f;
    for (float end = t + beat_period; t < end; t += dt) {
        kin = straightStep(kin, dt);
        body = updateSharkBody(body, kin, params, dt);
        peak = std::fmax(peak, std::abs(frame.across(body.spine.back())));
    }
    CHECK(peak == doctest::Approx(0.30).epsilon(0.20));
}

TEST_CASE("straight at cruise with uneven dt, peak tail-tip offset is "
          "0.30 m +/- 20 %") {
    const BodyParams params = bodyParams(WAVE_AMPLITUDE);
    const float uneven[] = {1.0f / 30.0f, 0.007f, 1.0f / 144.0f, 0.019f,
                            0.001f};
    Kinematics kin = kinAt({2.0f, 3.0f}, -1.1f, 1.0f);
    const Vector2 origin = kin.pos, fwd = kin.heading;
    SharkBody body = seedSharkBody(kin, params);
    float t = 0.0f;
    int step = 0;
    double peak = 0.0;
    while (t < L + 2.0f) {
        float dt = uneven[step++ % 5];
        t += dt;
        kin = straightStep(kin, dt);
        body = updateSharkBody(body, kin, params, dt);
        if (t >= L)
            peak = std::fmax(peak, std::abs(lateral(body.spine.back(), origin, fwd)));
    }
    CHECK(peak == doctest::Approx(0.30).epsilon(0.20));
}

TEST_CASE("uneven dt (all <= 1/30 s) keeps the phase advance and the segment "
          "lengths") {
    const BodyParams params = bodyParams(WAVE_AMPLITUDE);
    const std::vector<float> uneven = {1.0f / 30.0f, 0.004f, 1.0f / 144.0f,
                                       0.021f,       0.0005f, 1.0f / 33.0f,
                                       0.0f,         0.012f};
    for (float v : {0.5f, 1.0f, 2.5f}) {
        CAPTURE(v);
        Kinematics kin = kinAt({15.0f, 8.0f}, 1.0f, v);
        SharkBody body = seedSharkBody(kin, params);
        for (int step = 0; step < 900; ++step) {
            CAPTURE(step);
            float dt = uneven[step % uneven.size()];
            float r = step % 300 < 100 ? 0.0f : (step % 300 < 200 ? 0.8f : -0.8f);
            kin = turnStep(kin, r, dt);
            double expected = body.phase + PI_D * v * dt;
            body = updateSharkBody(body, kin, params, dt);
            CHECK(std::abs(angleDiff(body.phase, expected)) <= 1e-6);
            checkSegmentLengths(body);
        }
    }
}

TEST_CASE("the drawn nose sways about the nose by A*L*amp(0) = 0.06 m") {
    const BodyParams params = bodyParams(WAVE_AMPLITUDE);
    const float dt = 1.0f / 60.0f;
    Kinematics kin = kinAt({5.0f, 8.0f}, 0.5f, 1.0f);
    SharkBody body = seedSharkBody(kin, params);
    double peak = 0.0;
    for (int step = 0; step < 600; ++step) {
        kin = straightStep(kin, dt);
        body = updateSharkBody(body, kin, params, dt);
        double sway = std::hypot(body.spine[0].x - kin.pos.x,
                                 body.spine[0].y - kin.pos.y);
        CHECK(sway <= 0.06 + 1e-4);
        peak = std::fmax(peak, sway);
    }
    CHECK(peak == doctest::Approx(0.06).epsilon(0.02));
}

}
