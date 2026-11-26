#include "doctest/doctest.h"

#include "shark_body.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace {

constexpr float L = 3.0f;
constexpr float MM = 0.001f;
constexpr int NODES = 24;

constexpr float KNOT_S[] = {0.00f, 0.10f, 0.25f, 0.40f,
                            0.60f, 0.75f, 0.90f, 1.00f};
constexpr float KNOT_WIDTH_OVER_L[] = {0.05f, 0.09f, 0.12f, 0.11f,
                            0.07f, 0.035f, 0.02f, 0.005f};

BodyParams reviewParams() {
    return {.length = L,
            .node_count = NODES,
            .wave_amplitude = 0.1f,
            .wave_number = 5.3f,
            .strouhal = 0.3f,
            .length_passes = 1};
}

Kinematics kinHeading(Vector2 pos, float phi, float v, float r) {
    return {pos, phi, v, r, {std::cos(phi), -std::sin(phi)}};
}

float dist(Vector2 a, Vector2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

void checkWidthBracketed(float s, float width) {
    int k = 0;
    while (k + 2 < 8 && KNOT_S[k + 1] <= s)
        ++k;
    float lo = std::min(KNOT_WIDTH_OVER_L[k], KNOT_WIDTH_OVER_L[k + 1]) * L;
    float hi = std::max(KNOT_WIDTH_OVER_L[k], KNOT_WIDTH_OVER_L[k + 1]) * L;
    CHECK(width >= lo - MM);
    CHECK(width <= hi + MM);
}

SharkBody swimTurning(float r, int frames) {
    const BodyParams params = reviewParams();
    const float v = 1.0f, dt = 1.0f / 60.0f;
    Kinematics kin = kinHeading({15.0f, 8.0f}, 0.3f, v, r);
    SharkBody body = seedSharkBody(kin, params);
    for (int f = 0; f < frames; ++f) {
        float phi = kin.phi + r * dt;
        Vector2 pos = {kin.pos.x + kin.heading.x * v * dt,
                       kin.pos.y + kin.heading.y * v * dt};
        kin = kinHeading(pos, phi, v, r);
        body = updateSharkBody(body, kin, params, dt);
    }
    return body;
}

} // namespace

TEST_CASE("review 04 / R13: on a waved, turning 24-node spine the outline is "
          "symmetric about each node and normal to the local chord") {
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

TEST_CASE("review 04 / R13, R14: end widths are w(0) and w(1) on the real "
          "node count") {
    SharkBody body = swimTurning(0.5f, 120);
    BodyShape shape = buildBodyShape(body.spine, L);
    CHECK(std::abs(dist(shape.left[0], shape.right[0]) - 0.05f * L) <= MM);
    CHECK(std::abs(dist(shape.left[NODES - 1], shape.right[NODES - 1]) -
                   0.005f * L) <= MM);
    REQUIRE(shape.rigid_parts.size() == BodyShape::RIGID_PART_COUNT);
    const int last_node_before_caudal = 17;
    const Vector2 c = body.spine[last_node_before_caudal];
    const Vector2 fwd = {body.spine[last_node_before_caudal - 1].x -
                             body.spine[last_node_before_caudal + 1].x,
                         body.spine[last_node_before_caudal - 1].y -
                             body.spine[last_node_before_caudal + 1].y};
    for (const std::vector<Vector2> &part : shape.rigid_parts)
        for (Vector2 v : part)
            CHECK((v.x - c.x) * fwd.x + (v.y - c.y) * fwd.y > 0.0f);
}

TEST_CASE("review 04 / R13: left is the screen-left of the swimming "
          "direction") {
    std::vector<Vector2> spine(NODES);
    for (int i = 0; i < NODES; ++i)
        spine[i] = {10.0f - L * i / (NODES - 1), 5.0f};
    BodyShape shape = buildBodyShape(spine, L);
    for (int i = 0; i < NODES; ++i) {
        CAPTURE(i);
        CHECK(shape.left[i].y < spine[i].y);
        CHECK(shape.right[i].y > spine[i].y);
    }
}
