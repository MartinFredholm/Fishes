#include "doctest/doctest.h"

#include "shark_body.h"
#include <cmath>

namespace {

constexpr float L = 3.0f;
constexpr int NODES = 24;

BodyParams params() {
    return {.length = L,
            .node_count = NODES,
            .wave_amplitude = 0.0f,
            .wave_number = 5.3f,
            .strouhal = 0.3f,
            .length_passes = 1};
}

Kinematics kinAt(Vector2 pos, float phi, float v) {
    return {pos, phi, v, 0.0f, {std::cos(phi), -std::sin(phi)}};
}

float dist(Vector2 a, Vector2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

Kinematics turnStep(const Kinematics &k, float r, float dt) {
    Vector2 pos = {k.pos.x + k.heading.x * k.v * dt,
                   k.pos.y + k.heading.y * k.v * dt};
    Kinematics n = kinAt(pos, k.phi + r * dt, k.v);
    n.r = r;
    return n;
}

} // namespace

TEST_CASE("review 01: default heading of an aggregate-initialised Kinematics "
          "matches phi (randomStart relies on it)") {
    Kinematics k{{1.0f, 2.0f}, 1.1f, 1.0f, 0.0f};
    CHECK(k.heading.x == doctest::Approx(std::cos(1.1f)));
    CHECK(k.heading.y == doctest::Approx(-std::sin(1.1f)));
}

TEST_CASE("review 01 R4: spine node 0 is kin.pos every frame") {
    Kinematics kin = kinAt({5.0f, 5.0f}, 0.0f, 1.0f);
    SharkBody body = seedSharkBody(kin, params());
    CHECK(dist(body.spine[0], kin.pos) < 1e-5f);
    for (int k = 0; k < 200; ++k) {
        kin = turnStep(kin, 0.8f, 1.0f / 60.0f);
        body = updateSharkBody(body, kin, params(), 1.0f / 60.0f);
        CHECK(dist(body.spine[0], kin.pos) < 1e-5f);
    }
}
