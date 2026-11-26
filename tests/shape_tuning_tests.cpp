#include "doctest/doctest.h"

#include "shark_body.h"
#include <cmath>
#include <vector>

namespace {

constexpr float L = 3.0f;
constexpr int NODES = 24;

float cross(Vector2 o, Vector2 a, Vector2 b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

bool insideTriangle(Vector2 p, Vector2 a, Vector2 b, Vector2 c) {
    float d1 = cross(a, b, p), d2 = cross(b, c, p), d3 = cross(c, a, p);
    return (d1 > 0 && d2 > 0 && d3 > 0) || (d1 < 0 && d2 < 0 && d3 < 0);
}

bool insideFan(Vector2 p, const std::vector<Vector2> &fan) {
    for (size_t i = 1; i + 1 < fan.size(); ++i)
        if (insideTriangle(p, fan[0], fan[i], fan[i + 1]))
            return true;
    return false;
}

void checkHeadCovered(const std::vector<Vector2> &spine) {
    BodyShape shape = buildBodyShape(spine, L);
    REQUIRE(shape.rigid_parts.size() == BodyShape::RIGID_PART_COUNT);
    const std::vector<Vector2> &head = shape.rigid_parts[BodyShape::CEPHALOFOIL];
    CHECK(insideFan(shape.left[0], head));
    CHECK(insideFan(shape.right[0], head));
}

std::vector<Vector2> arcSpine(float radius, float bend) {
    std::vector<Vector2> nodes;
    for (int i = 0; i < NODES; ++i) {
        float theta = bend * (float)i / (NODES - 1) * L / radius;
        nodes.push_back({15.0f + radius * std::cos(theta),
                         8.0f + radius * std::sin(theta)});
    }
    return nodes;
}

Kinematics kinAt(Vector2 pos, float phi, float v, float r) {
    return {pos, phi, v, r, {std::cos(phi), -std::sin(phi)}};
}

} // namespace

TEST_CASE("06: the cephalofoil covers the front of the body outline on "
          "straight and bent spines") {
    std::vector<Vector2> straight;
    for (int i = 0; i < NODES; ++i)
        straight.push_back({10.0f - L * (float)i / (NODES - 1), 5.0f});
    checkHeadCovered(straight);

    for (float radius : {3.0f, 1.5f, 0.625f})
        for (float bend : {1.0f, -1.0f}) {
            CAPTURE(radius);
            CAPTURE(bend);
            checkHeadCovered(arcSpine(radius, bend));
        }
}

TEST_CASE("06: the cephalofoil covers the front of the body outline while "
          "swimming and turning") {
    const BodyParams params = {.length = L,
                               .node_count = NODES,
                               .wave_amplitude = 0.1f,
                               .wave_number = 5.3f,
                               .strouhal = 0.3f,
                               .length_passes = 1};
    const float dt = 1.0f / 60.0f;
    for (float v : {0.5f, 1.0f, 2.5f})
        for (float r : {0.0f, 0.8f, -0.8f}) {
            CAPTURE(v);
            CAPTURE(r);
            Kinematics kin = kinAt({15.0f, 8.0f}, 0.3f, v, r);
            SharkBody body = seedSharkBody(kin, params);
            for (int f = 0; f < 300; ++f) {
                Vector2 pos = {kin.pos.x + kin.heading.x * v * dt,
                               kin.pos.y + kin.heading.y * v * dt};
                kin = kinAt(pos, kin.phi + r * dt, v, r);
                body = updateSharkBody(body, kin, params, dt);
                if (f % 7 == 0)
                    checkHeadCovered(body.spine);
            }
        }
}
