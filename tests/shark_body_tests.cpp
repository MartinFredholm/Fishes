#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "shark_body.h"
#include <cmath>

namespace {

constexpr float L = 3.0f;
constexpr int NODES = 24;
constexpr float TOL_M = 1e-3f;

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

float dist(Vector2 a, Vector2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

float nodeS(int i) { return (float)i / (NODES - 1); }

Kinematics straightStep(const Kinematics &kin, float dt) {
    return kinAt({kin.pos.x + kin.heading.x * kin.v * dt,
                  kin.pos.y + kin.heading.y * kin.v * dt},
                 kin.phi, kin.v);
}

// y down: pos = c + R·(sin phi, cos phi) moves along heading
// (cos phi, −sin phi).
struct Circle {
    Vector2 c;
    float R;
    Kinematics at(float phi, float v) const {
        return kinAt({c.x + R * std::sin(phi), c.y + R * std::cos(phi)}, phi,
                     v);
    }
    float angleOf(Vector2 p) const { return std::atan2(p.x - c.x, p.y - c.y); }
};

float wrapAngle(float a) { return std::remainder(a, 2.0f * (float)M_PI); }

struct CircleRun {
    SharkBody body;
    float phi;
};

CircleRun swimCircle(const Circle &circle, float phi0, float v,
                     const std::vector<float> &dts, float distance) {
    const BodyParams params = flatBody();
    float phi = phi0;
    SharkBody body = seedSharkBody(circle.at(phi, v), params);
    float swum = 0.0f;
    for (size_t k = 0; swum < distance; ++k) {
        float dt = dts[k % dts.size()];
        phi += v / circle.R * dt;
        swum += v * dt;
        body = updateSharkBody(body, circle.at(phi, v), params, dt);
    }
    return {body, phi};
}

void checkSpineOnCircle(const Circle &circle, const CircleRun &run) {
    REQUIRE(run.body.spine.size() == NODES);
    for (int i = 0; i < NODES; ++i) {
        CAPTURE(i);
        Vector2 node = run.body.spine[i];
        CHECK(std::abs(dist(node, circle.c) - circle.R) < TOL_M);
        float arc_behind =
            circle.R * wrapAngle(run.phi - circle.angleOf(node));
        CHECK(std::abs(arc_behind - nodeS(i) * L) < TOL_M);
    }
}

} // namespace

TEST_CASE("R6: seeded spine is straight, length L behind the nose") {
    const float phi = 0.7f;
    Kinematics kin = kinAt({10.0f, 5.0f}, phi, 1.0f);
    SharkBody body = seedSharkBody(kin, flatBody());

    REQUIRE(body.spine.size() == NODES);
    Vector2 back = {-std::cos(phi), std::sin(phi)};
    for (int i = 0; i < NODES; ++i) {
        Vector2 expected = {10.0f + back.x * nodeS(i) * L,
                            5.0f + back.y * nodeS(i) * L};
        CHECK(dist(body.spine[i], expected) < TOL_M);
    }
}

TEST_CASE("R5: path keeps at least L behind the nose and trims older points") {
    const BodyParams params = flatBody();
    Kinematics kin = kinAt({5.0f, 5.0f}, 0.3f, 1.0f);
    SharkBody body = seedSharkBody(kin, params);
    const float dt = 1.0f / 60.0f;

    for (int step = 0; step < 600; ++step) {
        kin = straightStep(kin, dt);
        body = updateSharkBody(body, kin, params, dt);

        REQUIRE(body.path.size() >= 2);
        const PathPoint &nose = body.path.back();
        CHECK(dist(nose.pos, kin.pos) < TOL_M);
        CHECK(nose.arc - body.path.front().arc >= L);
        CHECK(nose.arc - body.path[1].arc < L);
    }
    CHECK(body.path.size() < 200);
}

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
        CHECK(std::abs(across) < TOL_M);
        float nose_along = (kin.pos.x - start.x) * fwd.x +
                           (kin.pos.y - start.y) * fwd.y;
        CHECK(std::abs((nose_along - along) - nodeS(i) * L) < TOL_M);
    }
}

TEST_CASE("R7: zero amplitude, circular path: nodes on the circle at s_i L") {
    const Circle circle{{8.0f, 6.0f}, 1.0f};
    CircleRun run = swimCircle(circle, 0.4f, 1.0f, {1.0f / 60.0f}, 2.0f * L);
    checkSpineOnCircle(circle, run);
}

TEST_CASE("R12: uneven dt (all <= 1/30 s) still meets R7") {
    const std::vector<float> uneven = {1.0f / 30.0f, 0.004f, 1.0f / 144.0f,
                                       0.021f,       0.0005f, 1.0f / 33.0f,
                                       0.012f};
    const Circle circle{{8.0f, 6.0f}, 1.0f};
    CircleRun run = swimCircle(circle, -2.0f, 1.0f, uneven, 2.0f * L);
    checkSpineOnCircle(circle, run);

    CircleRun fast = swimCircle(circle, -2.0f, 2.5f, uneven, 2.0f * L);
    checkSpineOnCircle(circle, fast);
}
