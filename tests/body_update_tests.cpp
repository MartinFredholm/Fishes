#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "test_support.h"
#include <cmath>
#include <vector>

namespace {

constexpr float TOL_M = 1e-3f;

float nodeS(int i) { return (float)i / (NODES - 1); }

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

CircleRun swimCircle(const Circle &circle, float phi0, float v, float dt,
                     float distance) {
    const BodyParams params = bodyParams(0.0f);
    float phi = phi0;
    SharkBody body = seedSharkBody(circle.at(phi, v), params);
    for (float swum = 0.0f; swum < distance; swum += v * dt) {
        phi += v / circle.R * dt;
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
        CHECK(on.distance < TOL_M);
        CHECK(std::abs(on.arc_behind - nodeS(i) * L) < TOL_M);
    }
}

struct Projection {
    float distance;
    float arc;
};

struct ReferencePath {
    std::vector<Vector2> pts;
    std::vector<double> arc;

    void add(Vector2 p) {
        if (pts.empty()) {
            pts.push_back(p);
            arc.push_back(0.0);
            return;
        }
        double d = std::hypot((double)p.x - pts.back().x,
                              (double)p.y - pts.back().y);
        pts.push_back(p);
        arc.push_back(arc.back() + d);
    }

    Projection project(Vector2 q) const {
        Projection best{1e9f, 0.0f};
        for (size_t k = 1; k < pts.size(); ++k) {
            double ax = pts[k - 1].x, ay = pts[k - 1].y;
            double bx = pts[k].x - ax, by = pts[k].y - ay;
            double len2 = bx * bx + by * by;
            double t = len2 > 0 ? ((q.x - ax) * bx + (q.y - ay) * by) / len2 : 0;
            t = std::fmin(1.0, std::fmax(0.0, t));
            double d = std::hypot(q.x - (ax + t * bx), q.y - (ay + t * by));
            if (d < best.distance)
                best = {(float)d,
                        (float)(arc[k - 1] + t * (arc[k] - arc[k - 1]))};
        }
        return best;
    }
};

void checkOnReference(const SharkBody &body, const ReferencePath &ref) {
    REQUIRE(body.spine.size() == NODES);
    double nose_arc = ref.arc.back();
    for (int i = 0; i < NODES; ++i) {
        CAPTURE(i);
        Projection p = ref.project(body.spine[i]);
        CHECK(p.distance < TOL_M);
        CHECK(std::abs((float)(nose_arc - p.arc) - nodeS(i) * L) < TOL_M);
    }
}

} // namespace

TEST_SUITE("body update") {

TEST_CASE("aggregate-initialised Kinematics gets its heading from phi") {
    Kinematics k{{1.0f, 2.0f}, 1.1f, 1.0f, 0.0f};
    CHECK(k.heading.x == doctest::Approx(std::cos(1.1f)));
    CHECK(k.heading.y == doctest::Approx(-std::sin(1.1f)));
}

TEST_CASE("seeded spine is straight, length L behind the nose") {
    const float phi = 0.7f;
    Kinematics kin = kinAt({10.0f, 5.0f}, phi, 1.0f);
    SharkBody body = seedSharkBody(kin, bodyParams(0.0f));

    REQUIRE(body.spine.size() == NODES);
    Vector2 back = {-std::cos(phi), std::sin(phi)};
    for (int i = 0; i < NODES; ++i) {
        Vector2 expected = {10.0f + back.x * nodeS(i) * L,
                            5.0f + back.y * nodeS(i) * L};
        CHECK(dist(body.spine[i], expected) < TOL_M);
    }
}

TEST_CASE("first update after spawn keeps the spine straight along the "
          "initial heading") {
    const BodyParams params = bodyParams(0.0f);
    const float phi = 2.3f;
    Kinematics kin = kinAt({7.0f, 4.0f}, phi, 1.0f);
    SharkBody body = seedSharkBody(kin, params);
    kin = straightStep(kin, 1.0f / 60.0f);
    body = updateSharkBody(body, kin, params, 1.0f / 60.0f);
    Vector2 back = {-std::cos(phi), std::sin(phi)};
    REQUIRE(body.spine.size() == NODES);
    for (int i = 0; i < NODES; ++i) {
        Vector2 expected = {kin.pos.x + back.x * nodeS(i) * L,
                            kin.pos.y + back.y * nodeS(i) * L};
        CHECK(dist(body.spine[i], expected) < TOL_M);
    }
}

TEST_CASE("spine node 0 is the nose every frame") {
    const BodyParams params = bodyParams(0.0f);
    Kinematics kin = kinAt({5.0f, 5.0f}, 0.0f, 1.0f);
    SharkBody body = seedSharkBody(kin, params);
    CHECK(dist(body.spine[0], kin.pos) < 1e-5f);
    for (int k = 0; k < 200; ++k) {
        kin = turnStep(kin, 0.8f, 1.0f / 60.0f);
        body = updateSharkBody(body, kin, params, 1.0f / 60.0f);
        CHECK(dist(body.spine[0], kin.pos) < 1e-5f);
    }
}

TEST_CASE("nose path keeps at least L behind the nose and trims older "
          "points") {
    const BodyParams params = bodyParams(0.0f);
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

TEST_CASE("tightest turn keeps at least L of nose path, trims, and the spine "
          "lies on the path") {
    const BodyParams params = bodyParams(0.0f);
    Kinematics kin = kinAt({10.0f, 8.0f}, 0.5f, 0.5f);
    SharkBody body = seedSharkBody(kin, params);
    const float dt = 1.0f / 30.0f;
    for (int k = 0; k < 900; ++k) {
        kin = turnStep(kin, 0.8f, dt);
        body = updateSharkBody(body, kin, params, dt);
        const auto &p = body.path;
        REQUIRE(p.size() >= 2);
        CHECK(p.back().arc - p.front().arc >= L);
        CHECK(p.back().arc - p[1].arc < L);
    }
    checkSpineOnRecordedPath(body);
}

TEST_CASE("zero amplitude, straight path: nodes on the path at s·L") {
    const BodyParams params = bodyParams(0.0f);
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

TEST_CASE("zero amplitude, circular path: nodes on the circle at s·L") {
    const Circle circle{{8.0f, 6.0f}, 1.0f};
    CircleRun run = swimCircle(circle, 0.4f, 1.0f, 1.0f / 60.0f, 2.0f * L);
    checkSpineOnCircle(circle, run);
}

TEST_CASE("varying speed and curvature with uneven dt: nodes on the traced "
          "path at s·L") {
    const BodyParams params = bodyParams(0.0f);
    const std::vector<float> dts = {1.0f / 30.0f, 0.003f, 1.0f / 90.0f, 0.0f,
                                    0.027f,       0.0001f, 1.0f / 45.0f};
    float phi = 0.9f;
    Kinematics kin = kinAt({15.0f, 8.0f}, phi, 1.0f);

    ReferencePath ref;
    Vector2 tail = {kin.pos.x - kin.heading.x * L,
                    kin.pos.y - kin.heading.y * L};
    ref.add(tail);
    ref.add(kin.pos);
    SharkBody body = seedSharkBody(kin, params);

    float t = 0.0f;
    for (int k = 0; k < 3000; ++k) {
        float dt = dts[k % dts.size()];
        t += dt;
        float v = 1.45f + 1.05f * std::sin(0.7f * t);
        float r = 0.8f * std::sin(0.45f * t);
        Vector2 pos = {kin.pos.x + kin.heading.x * kin.v * dt,
                       kin.pos.y + kin.heading.y * kin.v * dt};
        phi += kin.r * dt;
        kin = kinAt(pos, phi, v, r);
        body = updateSharkBody(body, kin, params, dt);
        ref.add(kin.pos);

        if (k % 250 == 0) {
            CAPTURE(k);
            checkOnReference(body, ref);
        }
    }
    checkOnReference(body, ref);
}

TEST_CASE("one-hour swim keeps the nose path bounded and the spine on it") {
    const BodyParams params = bodyParams(0.0f);
    Kinematics kin = kinAt({5.0f, 5.0f}, 0.0f, 1.0f);
    SharkBody body = seedSharkBody(kin, params);
    const float dt = 1.0f / 60.0f;
    for (int k = 0; k < 60 * 3600; ++k) {
        kin = turnStep(kin, 0.05f, dt);
        body = updateSharkBody(body, kin, params, dt);
    }
    CHECK(body.path.size() < 400);
    CHECK(body.path.back().arc < 2.0f * L);
    checkSpineOnRecordedPath(body);
}

TEST_CASE("very small steps keep the spine finite") {
    const BodyParams params = bodyParams(0.0f);
    Kinematics kin = kinAt({5.0f, 5.0f}, 0.0f, 0.5f);
    SharkBody body = seedSharkBody(kin, params);
    for (int k = 0; k < 50; ++k) {
        kin.pos.x += 1e-7f;
        body = updateSharkBody(body, kin, params, 1e-6f);
    }
    for (Vector2 p : body.spine) {
        CHECK(std::isfinite(p.x));
        CHECK(std::isfinite(p.y));
    }
}

}
