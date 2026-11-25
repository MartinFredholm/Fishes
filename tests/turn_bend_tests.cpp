#include "doctest/doctest.h"

#include "shark_body.h"
#include <cmath>
#include <limits>

namespace {

constexpr float L = 3.0f;
constexpr int NODES = 24;
constexpr float V_MIN = 0.5f, V_MAX = 2.5f, R_MAX = 0.8f;

BodyParams bodyWithAmplitude(float amplitude) {
    return {.length = L,
            .node_count = NODES,
            .wave_amplitude = amplitude,
            .wave_number = 5.3f,
            .strouhal = 0.3f,
            .length_passes = 1};
}

Kinematics makeKin(Vector2 pos, float phi, float v, float r) {
    return {pos, phi, v, r, {std::cos(phi), -std::sin(phi)}};
}

} // namespace

TEST_CASE("R6: first update after spawn at cruise moves no node more than "
          "0.05 m, and the seed already carries the wave at phase 0") {
    const BodyParams params = bodyWithAmplitude(0.1f);
    const float dt = 1.0f / 60.0f;
    for (float phi : {0.0f, 2.3f, -0.9f}) {
        CAPTURE(phi);
        Kinematics kin = makeKin({7.0f, 4.0f}, phi, 1.0f, 0.0f);
        SharkBody seed = seedSharkBody(kin, params);
        REQUIRE(seed.spine.size() == NODES);
        CHECK(seed.phase == 0.0f);

        SharkBody at_phase_zero;
        at_phase_zero.phase = 0.0f;
        SharkBody ordinary = updateSharkBody(at_phase_zero, kin, params, 0.0f);
        for (int i = 0; i < NODES; ++i) {
            CAPTURE(i);
            CHECK(std::hypot(seed.spine[i].x - ordinary.spine[i].x,
                             seed.spine[i].y - ordinary.spine[i].y) < 1e-6f);
        }

        Kinematics moved = makeKin({kin.pos.x + kin.heading.x * kin.v * dt,
                                    kin.pos.y + kin.heading.y * kin.v * dt},
                                   phi, 1.0f, 0.0f);
        SharkBody first = updateSharkBody(seed, moved, params, dt);
        for (int i = 0; i < NODES; ++i) {
            CAPTURE(i);
            CHECK(std::hypot(first.spine[i].x - seed.spine[i].x,
                             first.spine[i].y - seed.spine[i].y) <= 0.05f);
        }
    }
}

TEST_CASE("R20: v from 0 up to v_min at r = +-r_max keeps every coordinate "
          "finite") {
    const float speeds[] = {0.0f,  std::numeric_limits<float>::denorm_min(),
                            1e-38f, 1e-30f, 1e-12f, 1e-6f, 1e-3f, 0.05f,
                            0.25f,  V_MIN};
    for (float amplitude : {0.0f, 0.1f}) {
        const BodyParams params = bodyWithAmplitude(amplitude);
        for (float v : speeds) {
            for (float r : {R_MAX, -R_MAX}) {
                CAPTURE(amplitude);
                CAPTURE(v);
                CAPTURE(r);
                Kinematics kin = makeKin({6.0f, 9.0f}, 0.6f, v, r);
                SharkBody body = seedSharkBody(kin, params);
                for (int step = 0; step < 3; ++step)
                    body = updateSharkBody(body, kin, params, 1.0f / 60.0f);
                REQUIRE(body.spine.size() == NODES);
                for (Vector2 p : body.spine) {
                    CHECK(std::isfinite(p.x));
                    CHECK(std::isfinite(p.y));
                }
            }
        }
    }
}

TEST_CASE("R12: uneven dt with v and r changing every step keeps R9 and R10") {
    const BodyParams params = bodyWithAmplitude(0.1f);
    const float uneven[] = {1.0f / 30.0f, 0.0f,         0.004f, 1.0f / 144.0f,
                            0.021f,       0.0005f,      0.0f,   1.0f / 33.0f};
    const double pi = 3.14159265358979323846;
    const float rest = L / (NODES - 1);
    float phi = 0.3f, t = 0.0f;
    Kinematics kin = makeKin({15.0f, 8.0f}, phi, V_MIN, R_MAX);
    SharkBody body = seedSharkBody(kin, params);
    for (int step = 0; step < 1200; ++step) {
        CAPTURE(step);
        float dt = uneven[step % 8];
        t += dt;
        float v = V_MIN + (V_MAX - V_MIN) * 0.5f * (1.0f - std::cos(0.9f * t));
        float r = R_MAX * std::sin(1.7f * t + 1.5f);
        Vector2 pos = {kin.pos.x + kin.heading.x * kin.v * dt,
                       kin.pos.y + kin.heading.y * kin.v * dt};
        phi += kin.r * dt;
        kin = makeKin(pos, phi, v, r);

        double expected = body.phase + pi * v * dt;
        body = updateSharkBody(body, kin, params, dt);
        CHECK(std::abs(std::remainder(body.phase - expected, 2.0 * pi)) <= 1e-6);
        REQUIRE(body.spine.size() == NODES);
        for (int i = 0; i + 1 < NODES; ++i) {
            CAPTURE(i);
            float d = std::hypot(body.spine[i + 1].x - body.spine[i].x,
                                 body.spine[i + 1].y - body.spine[i].y);
            CHECK(std::abs(d - rest) <= 0.001f * rest);
        }
    }
}
