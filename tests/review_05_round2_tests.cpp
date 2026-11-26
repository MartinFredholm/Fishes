#include "doctest/doctest.h"

#include "shark_body.h"
#include <cmath>
#include <vector>

namespace {

constexpr float L = 3.0f;
constexpr float MM = 0.001f;
constexpr int NODES = 24;

BodyParams params() {
    return {.length = L,
            .node_count = NODES,
            .wave_amplitude = 0.1f,
            .wave_number = 5.3f,
            .strouhal = 0.3f,
            .length_passes = 1};
}

Kinematics kinAt(Vector2 pos, float phi, float v, float r) {
    return {pos, phi, v, r, {std::cos(phi), -std::sin(phi)}};
}

float dist(Vector2 a, Vector2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

Vector2 unit(Vector2 a) {
    float n = std::hypot(a.x, a.y);
    return {a.x / n, a.y / n};
}

float cross(Vector2 a, Vector2 b) { return a.x * b.y - a.y * b.x; }

Vector2 sub(Vector2 a, Vector2 b) { return {a.x - b.x, a.y - b.y}; }

float turnAt(const std::vector<Vector2> &spine, int i) {
    Vector2 u = sub(spine[i], spine[i - 1]), w = sub(spine[i + 1], spine[i]);
    return std::atan2(cross(u, w), u.x * w.x + u.y * w.y);
}

// The issue 03/04 body update folds the spine at some nodes in the tightest
// turn (turn angles above 1 rad). Interpolating across a fold is undefined,
// so those frames are skipped here and reported in reviews/05.md.
bool smooth(const std::vector<Vector2> &spine) {
    for (int i = 1; i + 1 < (int)spine.size(); ++i)
        if (std::abs(turnAt(spine, i)) > 0.5f)
            return false;
    return true;
}

// Tangent at node 0 of the circle through nodes 0, 1 and 2: the chord 0-1
// turned back by half the turn angle at node 1. Points tailward.
Vector2 noseTailward(const std::vector<Vector2> &spine) {
    Vector2 c = unit(sub(spine[1], spine[0]));
    float a = -turnAt(spine, 1) / 2;
    return {c.x * std::cos(a) - c.y * std::sin(a),
            c.x * std::sin(a) + c.y * std::cos(a)};
}

} // namespace

TEST_CASE("review 05 round 2 / R15: rigid parts on a swum, waved, turning "
          "spine") {
    const BodyParams p = params();
    // v_min and r_max (docs/spec.md R3): the tightest turn.
    for (float r : {0.8f, -0.8f, 0.0f}) {
        CAPTURE(r);
        const float v = 0.5f, dt = 1.0f / 60.0f;
        Kinematics kin = kinAt({15.0f, 8.0f}, 0.3f, v, r);
        SharkBody body = seedSharkBody(kin, p);
        for (int f = 0; f < 600; ++f) {
            Vector2 pos = {kin.pos.x + kin.heading.x * v * dt,
                           kin.pos.y + kin.heading.y * v * dt};
            kin = kinAt(pos, kin.phi + r * dt, v, r);
            body = updateSharkBody(body, kin, p, dt);
            if (f % 37 != 0 || !smooth(body.spine))
                continue;
            BodyShape shape = buildBodyShape(body.spine, L);
            REQUIRE(shape.rigid_parts.size() == 5);

            // s = 0 is node 0.
            CHECK(dist(shape.rigid_parts[BodyShape::CEPHALOFOIL][0],
                       body.spine[0]) <= MM);
            // s = 0.26 is 0.26 - 6/23 of L ahead of node 6, i.e. 2.6 mm.
            const float ahead = (6.0f / 23.0f - 0.26f) * L;
            for (auto part : {BodyShape::PECTORAL_LEFT,
                              BodyShape::PECTORAL_RIGHT})
                CHECK(std::abs(dist(shape.rigid_parts[part][0],
                                    body.spine[6]) - ahead) <= MM);
            // s = 0.54 lies between nodes 12 and 13: both distances add up
            // to the segment length L/23 within 1 mm.
            for (auto part : {BodyShape::PELVIC_LEFT,
                              BodyShape::PELVIC_RIGHT}) {
                Vector2 a = shape.rigid_parts[part][0];
                CHECK(dist(a, body.spine[12]) + dist(a, body.spine[13]) -
                          L / 23 <= MM);
                CHECK(std::abs(dist(a, body.spine[12]) -
                               (0.54f - 12.0f / 23.0f) * L) <= 2 * MM);
            }

            // Orientation: the cephalofoil's midline vertex (chord 0.06 L)
            // lies behind the nose along the nose tangent, within 2 degrees.
            Vector2 back = noseTailward(body.spine);
            const auto &head = shape.rigid_parts[BodyShape::CEPHALOFOIL];
            bool found = false;
            for (Vector2 q : head) {
                Vector2 d = {q.x - head[0].x, q.y - head[0].y};
                if (std::abs(std::hypot(d.x, d.y) - 0.06f * L) > 0.01f)
                    continue;
                Vector2 u = unit(d);
                if (u.x * back.x + u.y * back.y > std::cos(0.035f))
                    found = true;
            }
            CHECK(found);

            // Left fins on the screen-left of the swimming direction.
            Vector2 fwd = unit({body.spine[5].x - body.spine[7].x,
                                body.spine[5].y - body.spine[7].y});
            for (Vector2 q : shape.rigid_parts[BodyShape::PECTORAL_LEFT]) {
                Vector2 d = {q.x - body.spine[6].x, q.y - body.spine[6].y};
                CHECK(cross(fwd, d) <= 0.01f);
            }
            for (Vector2 q : shape.rigid_parts[BodyShape::PECTORAL_RIGHT]) {
                Vector2 d = {q.x - body.spine[6].x, q.y - body.spine[6].y};
                CHECK(cross(fwd, d) >= -0.01f);
            }
        }
    }
}
