#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "shark_body.h"
#include <cmath>

namespace {

constexpr float L = 3.0f;
constexpr int NODES = 24;
constexpr float TOL = 1e-3f; // m

BodyParams flatBody() {
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

float nodeS(int i) { return (float)i / (NODES - 1); }

Kinematics straightStep(const Kinematics &kin, float dt) {
    return kinAt({kin.pos.x + kin.heading.x * kin.v * dt,
                  kin.pos.y + kin.heading.y * kin.v * dt},
                 kin.phi, kin.v);
}

} // namespace

TEST_CASE("R7: zero amplitude, straight path: nodes on the path at s_i L") {
    const BodyParams params = flatBody();
    const float phi = -1.2f;
    Kinematics kin = kinAt({4.0f, 6.0f}, phi, 1.0f);
    const Vector2 start = kin.pos;
    SharkBody body = seedSharkBody(kin, params);
    const float dt = 1.0f / 60.0f;
    for (int step = 0; step < 300; ++step) {
        kin = straightStep(kin, dt);
        body = updateSharkBody(body, kin, params, dt);
    }

    REQUIRE(body.spine.size() == NODES);
    Vector2 fwd = {std::cos(phi), -std::sin(phi)};
    for (int i = 0; i < NODES; ++i) {
        CAPTURE(i);
        Vector2 rel = {body.spine[i].x - start.x, body.spine[i].y - start.y};
        float along = rel.x * fwd.x + rel.y * fwd.y;
        float across = rel.x * fwd.y - rel.y * fwd.x;
        CHECK(std::abs(across) < TOL);
        float nose_along = (kin.pos.x - start.x) * fwd.x +
                           (kin.pos.y - start.y) * fwd.y;
        CHECK(std::abs((nose_along - along) - nodeS(i) * L) < TOL);
    }
}
