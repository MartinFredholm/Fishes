#include "doctest/doctest.h"

#include "shark_body.h"
#include <cmath>

namespace {

constexpr float L = 3.0f;
constexpr int NODES = 24;
constexpr double PI_D = 3.14159265358979323846;

BodyParams waveBody() {
    return {.length = L,
            .node_count = NODES,
            .wave_amplitude = 0.1f,
            .wave_number = 5.3f,
            .strouhal = 0.3f,
            .length_passes = 1};
}

Kinematics kinAt(Vector2 pos, float phi, float v) {
    return {pos, phi, v, 0.0f, {std::cos(phi), -std::sin(phi)}};
}

Kinematics straightStep(const Kinematics &kin, float dt) {
    return kinAt({kin.pos.x + kin.heading.x * kin.v * dt,
                  kin.pos.y + kin.heading.y * kin.v * dt},
                 kin.phi, kin.v);
}

float dist(Vector2 a, Vector2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

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

Kinematics turnStep(const Kinematics &kin, float r, float dt) {
    Kinematics next = straightStep(kin, dt);
    next = kinAt(next.pos, kin.phi + r * dt, kin.v);
    next.r = r;
    return next;
}

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

} // namespace

TEST_CASE("R9: one update advances the phase by pi u dt") {
    const BodyParams params = waveBody();
    const float u = 1.7f;
    const float dt = 0.02f;
    Kinematics kin = kinAt({5.0f, 5.0f}, 0.3f, u);
    SharkBody body = seedSharkBody(kin, params);
    body.phase = 1.0f;

    SharkBody next = updateSharkBody(body, straightStep(kin, dt), params, dt);

    double expected = 1.0 + PI_D * u * dt;
    CHECK(std::abs(angleDiff(next.phase, expected)) <= 1e-6);
}

TEST_CASE("R8: wave offset at the blog's six amplitude points") {
    const double amp[] = {0.2, 0.21, 0.23, 0.4, 0.88, 1.0};
    BodyParams params = waveBody();
    params.node_count = 6;
    params.length_passes = 0;
    params.pivot = 1.0f / 3.0f;
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
            CHECK(std::abs(-frame.along(next.spine[i]) - (s - params.pivot) * L) <
                  1e-4);
        }
    }
}

TEST_CASE("R10: every segment within 0.1 % of L/23 after every update") {
    const BodyParams params = waveBody();
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

TEST_CASE("R11: straight at cruise, peak tail-tip offset over a beat is "
          "0.30 m +/- 20 %") {
    const BodyParams params = waveBody();
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

TEST_CASE("R12: uneven dt (all <= 1/30 s) still meets R9 and R10") {
    const BodyParams params = waveBody();
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
