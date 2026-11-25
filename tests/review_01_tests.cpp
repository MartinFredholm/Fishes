#include "doctest/doctest.h"

#include "shark_body.h"
#include <cmath>

namespace {

constexpr float L = 3.0f;
constexpr int NODES = 24;
constexpr float TOL = 1e-3f;

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

float distToSegment(Vector2 p, Vector2 a, Vector2 b) {
    float dx = b.x - a.x, dy = b.y - a.y;
    float len2 = dx * dx + dy * dy;
    float t = len2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0;
    t = std::fmax(0.0f, std::fmin(1.0f, t));
    return dist(p, {a.x + t * dx, a.y + t * dy});
}

struct OnPath {
    float distance;
    float arc_behind;
};

OnPath projectOnPath(const std::vector<PathPoint> &path, Vector2 p) {
    OnPath best{1e9f, 0};
    float behind = 0;
    for (int j = (int)path.size() - 1; j > 0; --j) {
        Vector2 a = path[j].pos, b = path[j - 1].pos;
        float d = distToSegment(p, a, b);
        if (d < best.distance)
            best = {d, behind + dist(a, p)};
        behind += dist(a, b);
    }
    return best;
}

void checkSpineOnRecordedPath(const SharkBody &body) {
    REQUIRE(body.spine.size() == NODES);
    for (int i = 0; i < NODES; ++i) {
        CAPTURE(i);
        OnPath on = projectOnPath(body.path, body.spine[i]);
        CHECK(on.distance < TOL);
        CHECK(std::abs(on.arc_behind - (float)i / (NODES - 1) * L) < TOL);
    }
}

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

TEST_CASE("review 01 R6: first update after spawn still shows a straight "
          "shark along the initial heading") {
    const float phi = 2.3f;
    Kinematics kin = kinAt({7.0f, 4.0f}, phi, 1.0f);
    SharkBody body = seedSharkBody(kin, params());
    kin = turnStep(kin, 0.0f, 1.0f / 60.0f);
    body = updateSharkBody(body, kin, params(), 1.0f / 60.0f);
    Vector2 back = {-std::cos(phi), std::sin(phi)};
    REQUIRE(body.spine.size() == NODES);
    for (int i = 0; i < NODES; ++i) {
        float s = (float)i / (NODES - 1);
        Vector2 expected = {kin.pos.x + back.x * s * L,
                            kin.pos.y + back.y * s * L};
        CHECK(dist(body.spine[i], expected) < TOL);
    }
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

TEST_CASE("review 01 R5/R7: tight turn at v_min / r_max keeps >= L of path, "
          "trims, and nodes sit on the recorded path") {
    Kinematics kin = kinAt({10.0f, 8.0f}, 0.5f, 0.5f);
    SharkBody body = seedSharkBody(kin, params());
    const float dt = 1.0f / 30.0f;
    for (int k = 0; k < 900; ++k) {
        kin = turnStep(kin, 0.8f, dt);
        body = updateSharkBody(body, kin, params(), dt);
        const auto &p = body.path;
        REQUIRE(p.size() >= 2);
        CHECK(p.back().arc - p.front().arc >= L);
        CHECK(p.back().arc - p[1].arc < L);
    }
    checkSpineOnRecordedPath(body);
}

TEST_CASE("review 01 R12: uneven dt including dt = 0 while turning") {
    const std::vector<float> dts = {1.0f / 30.0f, 0.0f, 0.001f, 1.0f / 144.0f,
                                    0.03f, 0.0002f, 1.0f / 60.0f};
    Kinematics kin = kinAt({10.0f, 8.0f}, -0.4f, 2.5f);
    SharkBody body = seedSharkBody(kin, params());
    for (int k = 0; k < 2000; ++k) {
        float dt = dts[k % dts.size()];
        float r = (k / 300) % 2 ? 0.8f : -0.5f;
        kin = turnStep(kin, r, dt);
        body = updateSharkBody(body, kin, params(), dt);
        REQUIRE(body.path.size() >= 2);
        CHECK(body.path.back().arc - body.path.front().arc >= L);
    }
    checkSpineOnRecordedPath(body);
}

TEST_CASE("review 01 R5: long swim keeps path bounded and spine exact") {
    Kinematics kin = kinAt({5.0f, 5.0f}, 0.0f, 1.0f);
    SharkBody body = seedSharkBody(kin, params());
    const float dt = 1.0f / 60.0f;
    for (int k = 0; k < 60 * 3600; ++k) {
        kin = turnStep(kin, 0.05f, dt);
        body = updateSharkBody(body, kin, params(), dt);
    }
    CHECK(body.path.size() < 400);
    CHECK(body.path.back().arc < 2.0f * L);
    checkSpineOnRecordedPath(body);
}
