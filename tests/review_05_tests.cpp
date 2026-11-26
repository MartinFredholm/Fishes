#include "doctest/doctest.h"

#include "shark_body.h"
#include <cmath>
#include <vector>

namespace {

constexpr float L = 3.0f;
constexpr float MM = 0.001f;
constexpr int NODES = 24;
constexpr float TIGHTEST_TURN_RADIUS = 0.625f;

float dist(Vector2 a, Vector2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

struct Arc {
    Vector2 centre;
    float radius;
    float bend;

    Vector2 at(float s) const {
        float theta = bend * s * L / radius;
        return {centre.x + radius * std::cos(theta),
                centre.y + radius * std::sin(theta)};
    }
    Vector2 forward(float s) const {
        float theta = bend * s * L / radius;
        return {bend * std::sin(theta), -bend * std::cos(theta)};
    }
    std::vector<Vector2> nodes() const {
        std::vector<Vector2> n;
        for (int i = 0; i < NODES; ++i)
            n.push_back(at((float)i / (NODES - 1)));
        return n;
    }
};

struct Attachment {
    BodyShape::RigidPart part;
    float s;
};

constexpr Attachment ATTACHMENTS[] = {
    {BodyShape::CEPHALOFOIL, 0.0f},
    {BodyShape::PECTORAL_LEFT, 0.26f},
    {BodyShape::PECTORAL_RIGHT, 0.26f},
    {BodyShape::PELVIC_LEFT, 0.54f},
    {BodyShape::PELVIC_RIGHT, 0.54f}};

} // namespace

TEST_CASE("review 05 / R15: on the 24-node spine, attachments lie on the arc "
          "within 1 mm") {
    for (float radius : {6.0f, 3.0f, 1.5f, TIGHTEST_TURN_RADIUS})
        for (float bend : {1.0f, -1.0f}) {
            Arc arc{{15.0f, 8.0f}, radius, bend};
            BodyShape shape = buildBodyShape(arc.nodes(), L);
            REQUIRE(shape.rigid_parts.size() == BodyShape::RIGID_PART_COUNT);
            for (Attachment a : ATTACHMENTS) {
                CAPTURE(radius);
                CAPTURE(bend);
                CAPTURE(a.s);
                CHECK(dist(shape.rigid_parts[a.part][0], arc.at(a.s)) <= MM);
            }
        }
}
