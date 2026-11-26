#include "doctest/doctest.h"

#include "test_support.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace {

struct WidthKnot {
    float s;
    float width_over_L;
};

constexpr std::array<WidthKnot, 8> WIDTH_PROFILE = {{{0.00f, 0.05f},
                                                     {0.10f, 0.09f},
                                                     {0.25f, 0.12f},
                                                     {0.40f, 0.11f},
                                                     {0.60f, 0.07f},
                                                     {0.75f, 0.035f},
                                                     {0.90f, 0.02f},
                                                     {1.00f, 0.005f}}};

constexpr int KNOT_NODES = 21;

int nodeAtS(float s, int nodes) {
    return (int)std::lround(s * (nodes - 1));
}

void checkOffset(Vector2 actual, Vector2 node, Vector2 dir, float offset) {
    Vector2 expected = {node.x + dir.x * offset, node.y + dir.y * offset};
    CHECK(dist(actual, expected) <= MM);
}

void checkWidthBracketed(float s, float width) {
    size_t k = 0;
    while (k + 2 < WIDTH_PROFILE.size() && WIDTH_PROFILE[k + 1].s <= s)
        ++k;
    float a = WIDTH_PROFILE[k].width_over_L * L;
    float b = WIDTH_PROFILE[k + 1].width_over_L * L;
    CHECK(width >= std::min(a, b) - MM);
    CHECK(width <= std::max(a, b) + MM);
}

SharkBody swimTurning(float r, int frames) {
    const BodyParams params = bodyParams(0.1f);
    const float dt = 1.0f / 60.0f;
    Kinematics kin = kinAt({15.0f, 8.0f}, 0.3f, 1.0f, r);
    SharkBody body = seedSharkBody(kin, params);
    for (int f = 0; f < frames; ++f) {
        kin = turnStep(kin, r, dt);
        body = updateSharkBody(body, kin, params, dt);
    }
    return body;
}

} // namespace

TEST_SUITE("body outline") {

TEST_CASE("straight spine: outline at ±w/2 on the screen-left normal") {
    const Vector2 fwd = {0.6f, -0.8f};
    const Vector2 screen_left = {fwd.y, -fwd.x};
    std::vector<Vector2> spine =
        straightSpine({12.0f, 7.0f}, fwd, KNOT_NODES);
    BodyShape shape = buildBodyShape(spine, L);
    REQUIRE(shape.left.size() == spine.size());
    REQUIRE(shape.right.size() == spine.size());
    for (const WidthKnot &k : WIDTH_PROFILE) {
        CAPTURE(k.s);
        int i = nodeAtS(k.s, KNOT_NODES);
        float half = k.width_over_L * L / 2;
        checkOffset(shape.left[i], spine[i], screen_left, half);
        checkOffset(shape.right[i], spine[i], screen_left, -half);
    }
}

TEST_CASE("arc spine: outline at ±w/2 on the radial normal") {
    const Vector2 centre = {15.0f, 8.0f};
    for (float radius : {3.0f, 1.5f, TIGHTEST_TURN_RADIUS})
        for (float bend : {1.0f, -1.0f}) {
            CAPTURE(radius);
            CAPTURE(bend);
            std::vector<Vector2> spine =
                arcSpine(centre, radius, bend, KNOT_NODES);
            BodyShape shape = buildBodyShape(spine, L);
            REQUIRE(shape.left.size() == spine.size());
            REQUIRE(shape.right.size() == spine.size());
            for (const WidthKnot &k : WIDTH_PROFILE) {
                CAPTURE(k.s);
                int i = nodeAtS(k.s, KNOT_NODES);
                Vector2 outward = {(spine[i].x - centre.x) / radius,
                                   (spine[i].y - centre.y) / radius};
                float half = bend * k.width_over_L * L / 2;
                checkOffset(shape.left[i], spine[i], outward, -half);
                checkOffset(shape.right[i], spine[i], outward, half);
            }
        }
}

TEST_CASE("waved, turning spine: the outline is symmetric about each node "
          "and normal to the local chord") {
    for (float r : {0.8f, -0.8f, 0.0f}) {
        CAPTURE(r);
        SharkBody body = swimTurning(r, 200);
        BodyShape shape = buildBodyShape(body.spine, L);
        REQUIRE(shape.left.size() == NODES);
        REQUIRE(shape.right.size() == NODES);
        for (int i = 1; i + 1 < NODES; ++i) {
            CAPTURE(i);
            Vector2 mid = {(shape.left[i].x + shape.right[i].x) / 2,
                           (shape.left[i].y + shape.right[i].y) / 2};
            CHECK(dist(mid, body.spine[i]) <= MM);
            Vector2 chord = {body.spine[i - 1].x - body.spine[i + 1].x,
                             body.spine[i - 1].y - body.spine[i + 1].y};
            float len = std::hypot(chord.x, chord.y);
            Vector2 off = {shape.left[i].x - body.spine[i].x,
                           shape.left[i].y - body.spine[i].y};
            CHECK(std::abs((off.x * chord.x + off.y * chord.y) / len) <= MM);
            float s = (float)i / (NODES - 1);
            checkWidthBracketed(s, dist(shape.left[i], shape.right[i]));
        }
    }
}

TEST_CASE("the outline tapers to w(1) at the tail tip, with no caudal "
          "shape") {
    std::vector<Vector2> spine = straightSpine({5.0f, 5.0f}, {1.0f, 0.0f});
    BodyShape shape = buildBodyShape(spine, L);
    REQUIRE(shape.left.size() == NODES);
    REQUIRE(shape.right.size() == NODES);
    REQUIRE(shape.rigid_parts.size() == BodyShape::RIGID_PART_COUNT);
    const float caudal_start_x = 5.0f - 0.75f * L;
    for (const std::vector<Vector2> &part : shape.rigid_parts)
        for (Vector2 v : part)
            CHECK(v.x > caudal_start_x);

    const float tail_width = WIDTH_PROFILE.back().width_over_L * L;
    CHECK(std::abs(dist(shape.left[NODES - 1], shape.right[NODES - 1]) -
                   tail_width) <= MM);

    const int caudal_start = (int)std::ceil(0.75f * (NODES - 1));
    for (int i = caudal_start; i + 1 < NODES; ++i) {
        CAPTURE(i);
        CHECK(dist(shape.left[i + 1], shape.right[i + 1]) <
              dist(shape.left[i], shape.right[i]));
    }
}

TEST_CASE("the outline is in world meters") {
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

}
