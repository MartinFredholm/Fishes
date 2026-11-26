#include "doctest/doctest.h"

#include "shark_body.h"
#include <cmath>
#include <vector>

namespace {

constexpr float L = 3.0f;
constexpr int NODES = 24;
constexpr float MM = 0.001f;

float signedArea(Vector2 a, Vector2 b, Vector2 c) {
    return 0.5f * ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
}

float polygonArea(const std::vector<Vector2> &p) {
    float sum = 0.0f;
    for (size_t i = 0; i < p.size(); ++i) {
        const Vector2 &a = p[i], &b = p[(i + 1) % p.size()];
        sum += a.x * b.y - b.x * a.y;
    }
    return 0.5f * std::abs(sum);
}

std::vector<Vector2> straightSpine() {
    std::vector<Vector2> nodes;
    for (int i = 0; i < NODES; ++i)
        nodes.push_back({10.0f - L * (float)i / (NODES - 1), 5.0f});
    return nodes;
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

void checkFansWoundLikeStrip(const std::vector<Vector2> &spine) {
    BodyShape shape = buildBodyShape(spine, L);
    float strip_sign =
        signedArea(shape.right[0], shape.left[0], shape.right[1]) > 0 ? 1.0f
                                                                      : -1.0f;
    for (size_t k = 0; k < shape.rigid_parts.size(); ++k) {
        CAPTURE(k);
        const std::vector<Vector2> &fan = shape.rigid_parts[k];
        float fan_area = 0.0f;
        for (size_t i = 1; i + 1 < fan.size(); ++i) {
            float a = signedArea(fan[0], fan[i], fan[i + 1]) * strip_sign;
            CHECK(a >= -1e-6f);
            fan_area += a;
        }
        std::vector<Vector2> rim = fan;
        if (k == BodyShape::CEPHALOFOIL)
            rim.assign(fan.begin() + 1, fan.end() - 1);
        CHECK(std::abs(fan_area - polygonArea(rim)) <= 1e-4f);
    }
}

} // namespace

TEST_CASE("review 06 / R15, R17: every fan triangle is wound like the outline "
          "strip, and the cephalofoil fan has no overlap") {
    checkFansWoundLikeStrip(straightSpine());
    for (float radius : {3.0f, 0.625f})
        for (float bend : {1.0f, -1.0f}) {
            CAPTURE(radius);
            CAPTURE(bend);
            checkFansWoundLikeStrip(arcSpine(radius, bend));
        }
}

TEST_CASE("review 06 / shape values: the cephalofoil reaches 0.01 L ahead of "
          "the nose, spans 0.27 L and has a 0.06 L midline chord") {
    std::vector<Vector2> spine = straightSpine();
    BodyShape shape = buildBodyShape(spine, L);
    const std::vector<Vector2> &head = shape.rigid_parts[BodyShape::CEPHALOFOIL];
    float front = -1e9f, min_y = 1e9f, max_y = -1e9f;
    for (Vector2 p : head) {
        front = std::max(front, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }
    CHECK(std::abs(front - (spine[0].x + 0.03f)) <= MM);
    CHECK(std::abs((max_y - min_y) - 0.27f * L) <= MM);

    float chord_back = -1e9f;
    for (Vector2 p : head)
        if (std::abs(p.y - spine[0].y) <= MM)
            chord_back = std::max(chord_back, spine[0].x - p.x);
    CHECK(std::abs(chord_back + 0.03f - 0.06f * L) <= MM);
}
