#include "doctest/doctest.h"

#include "shark_body.h"
#include <array>
#include <cmath>
#include <vector>

namespace {

constexpr float L = 3.0f;
constexpr float MM = 0.001f;

struct WidthKnot {
    float s;
    float width_over_L;
};

// docs/spec.md, shape values table.
constexpr std::array<WidthKnot, 8> SPEC_WIDTH = {{{0.00f, 0.05f},
                                                  {0.10f, 0.09f},
                                                  {0.25f, 0.12f},
                                                  {0.40f, 0.11f},
                                                  {0.60f, 0.07f},
                                                  {0.75f, 0.035f},
                                                  {0.90f, 0.02f},
                                                  {1.00f, 0.005f}}};

// 21 nodes put s_i = i/20 on every knot of the width table.
constexpr int KNOT_NODES = 21;

int nodeAtS(float s, int nodes) {
    return (int)std::lround(s * (nodes - 1));
}

float dist(Vector2 a, Vector2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

std::vector<Vector2> straightSpine(Vector2 nose, Vector2 fwd, int nodes) {
    std::vector<Vector2> spine(nodes);
    for (int i = 0; i < nodes; ++i) {
        float back = L * i / (nodes - 1);
        spine[i] = {nose.x - fwd.x * back, nose.y - fwd.y * back};
    }
    return spine;
}

// bend = +1 or -1 picks the side the arc curves to. Arc length from the nose
// to node i is s_i·L.
std::vector<Vector2> arcSpine(Vector2 centre, float radius, float bend,
                              int nodes) {
    std::vector<Vector2> spine(nodes);
    for (int i = 0; i < nodes; ++i) {
        float theta = bend * (L * i / (nodes - 1)) / radius;
        spine[i] = {centre.x + radius * std::cos(theta),
                    centre.y + radius * std::sin(theta)};
    }
    return spine;
}

void checkOffset(Vector2 actual, Vector2 node, Vector2 dir, float offset) {
    Vector2 expected = {node.x + dir.x * offset, node.y + dir.y * offset};
    CHECK(dist(actual, expected) <= MM);
}

} // namespace

TEST_CASE("R13: straight spine, outline at ±w/2 on the screen-left normal") {
    const Vector2 fwd = {0.6f, -0.8f};
    const Vector2 screen_left = {fwd.y, -fwd.x};
    std::vector<Vector2> spine =
        straightSpine({12.0f, 7.0f}, fwd, KNOT_NODES);
    BodyShape shape = buildBodyShape(spine, L);
    REQUIRE(shape.left.size() == spine.size());
    REQUIRE(shape.right.size() == spine.size());
    for (const WidthKnot &k : SPEC_WIDTH) {
        CAPTURE(k.s);
        int i = nodeAtS(k.s, KNOT_NODES);
        float half = k.width_over_L * L / 2;
        checkOffset(shape.left[i], spine[i], screen_left, half);
        checkOffset(shape.right[i], spine[i], screen_left, -half);
    }
}

TEST_CASE("R13: arc spine, outline at ±w/2 on the radial normal") {
    const Vector2 centre = {15.0f, 8.0f};
    // 0.625 m is the tightest turn radius, v_min / r_max (docs/spec.md).
    for (float radius : {3.0f, 1.5f, 0.625f})
        for (float bend : {1.0f, -1.0f}) {
            CAPTURE(radius);
            CAPTURE(bend);
            std::vector<Vector2> spine =
                arcSpine(centre, radius, bend, KNOT_NODES);
            BodyShape shape = buildBodyShape(spine, L);
            REQUIRE(shape.left.size() == spine.size());
            REQUIRE(shape.right.size() == spine.size());
            for (const WidthKnot &k : SPEC_WIDTH) {
                CAPTURE(k.s);
                int i = nodeAtS(k.s, KNOT_NODES);
                Vector2 outward = {(spine[i].x - centre.x) / radius,
                                   (spine[i].y - centre.y) / radius};
                // Swimming toward the nose, the centre is on screen-left when
                // bend = +1 (y down).
                float half = bend * k.width_over_L * L / 2;
                checkOffset(shape.left[i], spine[i], outward, -half);
                checkOffset(shape.right[i], spine[i], outward, half);
            }
        }
}

TEST_CASE("R14: the outline tapers to w(1) at the tail tip, no caudal shape") {
    constexpr int NODES = 24;
    std::vector<Vector2> spine =
        straightSpine({5.0f, 5.0f}, {1.0f, 0.0f}, NODES);
    BodyShape shape = buildBodyShape(spine, L);
    REQUIRE(shape.left.size() == NODES);
    REQUIRE(shape.right.size() == NODES);
    CHECK(shape.rigid_parts.empty());

    const float tail_width = SPEC_WIDTH.back().width_over_L * L;
    CHECK(std::abs(dist(shape.left[NODES - 1], shape.right[NODES - 1]) -
                   tail_width) <= MM);

    const int caudal_start = (int)std::ceil(0.75f * (NODES - 1));
    for (int i = caudal_start; i + 1 < NODES; ++i) {
        CAPTURE(i);
        CHECK(dist(shape.left[i + 1], shape.right[i + 1]) <
              dist(shape.left[i], shape.right[i]));
    }
}

TEST_CASE("R16: outline is in world meters") {
    const Vector2 nose = {29.0f, 15.0f};
    std::vector<Vector2> spine =
        straightSpine(nose, {1.0f, 0.0f}, KNOT_NODES);
    BodyShape shape = buildBodyShape(spine, L);
    int widest = nodeAtS(0.25f, KNOT_NODES);
    CHECK(std::abs(dist(shape.left[widest], shape.right[widest]) - 0.36f) <=
          MM);
    CHECK(std::abs(shape.left[0].x - nose.x) <= MM);
    CHECK(std::abs(shape.left[KNOT_NODES - 1].x - (nose.x - L)) <= MM);
}
