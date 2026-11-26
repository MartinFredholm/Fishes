#include "doctest/doctest.h"

#include "shark_body.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace {

constexpr float L = 3.0f;
constexpr float MM = 0.001f;
constexpr float TIGHTEST_TURN_RADIUS = 0.625f;
constexpr float LEFT = 1.0f;
constexpr float RIGHT = -1.0f;

constexpr int ANCHOR_NODES = 51;
constexpr int PRODUCTION_NODES = 24;

float dist(Vector2 a, Vector2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

struct KnownSpine {
    std::vector<Vector2> nodes;
    Vector2 centre{};
    float radius = 0.0f;
    float bend = 0.0f;
    Vector2 nose{};
    Vector2 fwd{};

    Vector2 at(float s) const {
        if (radius == 0.0f)
            return {nose.x - fwd.x * s * L, nose.y - fwd.y * s * L};
        float theta = bend * s * L / radius;
        return {centre.x + radius * std::cos(theta),
                centre.y + radius * std::sin(theta)};
    }
    Vector2 forward(float s) const {
        if (radius == 0.0f)
            return fwd;
        float theta = bend * s * L / radius;
        return {bend * std::sin(theta), -bend * std::cos(theta)};
    }
};

KnownSpine straight(Vector2 nose, Vector2 fwd, int node_count) {
    KnownSpine k;
    k.nose = nose;
    k.fwd = fwd;
    for (int i = 0; i < node_count; ++i)
        k.nodes.push_back(k.at((float)i / (node_count - 1)));
    return k;
}

KnownSpine arc(Vector2 centre, float radius, float bend, int node_count) {
    KnownSpine k;
    k.centre = centre;
    k.radius = radius;
    k.bend = bend;
    for (int i = 0; i < node_count; ++i)
        k.nodes.push_back(k.at((float)i / (node_count - 1)));
    return k;
}

std::vector<KnownSpine> testSpines() {
    std::vector<KnownSpine> spines;
    for (int nodes : {ANCHOR_NODES, PRODUCTION_NODES}) {
        spines.push_back(straight({12.0f, 7.0f}, {0.6f, -0.8f}, nodes));
        spines.push_back(straight({5.0f, 5.0f}, {-1.0f, 0.0f}, nodes));
        for (float radius : {3.0f, 1.5f, TIGHTEST_TURN_RADIUS})
            for (float bend : {1.0f, -1.0f})
                spines.push_back(arc({15.0f, 8.0f}, radius, bend, nodes));
    }
    return spines;
}

struct Local {
    float back;
    float left;
};

std::vector<Local> toLocal(const std::vector<Vector2> &part,
                           const KnownSpine &spine, float s) {
    Vector2 origin = spine.at(s);
    Vector2 fwd = spine.forward(s);
    Vector2 left = {fwd.y, -fwd.x};
    std::vector<Local> local;
    for (Vector2 p : part) {
        Vector2 d = {p.x - origin.x, p.y - origin.y};
        local.push_back({-(d.x * fwd.x + d.y * fwd.y),
                         d.x * left.x + d.y * left.y});
    }
    return local;
}

bool hasVertex(const std::vector<Local> &local, float back, float left) {
    return std::any_of(local.begin(), local.end(), [&](Local v) {
        return std::abs(v.back - back) <= MM && std::abs(v.left - left) <= MM;
    });
}

struct FinSpec {
    float root_s;
    float root_end_s;
    float tip_s;
    float span;
    float root_half_width_min;
    float root_half_width_max;
};

constexpr FinSpec PECTORAL = {0.26f, 0.32f, 0.38f, 0.14f, 0.11f / 2,
                              0.12f / 2};
constexpr FinSpec PELVIC = {0.54f, 0.57f, 0.60f, 0.04f, 0.07f / 2,
                            0.11f / 2};

void checkFin(const std::vector<Vector2> &part, const KnownSpine &spine,
              const FinSpec &fin, float side) {
    REQUIRE(part.size() >= 3);
    CHECK(dist(part[0], spine.at(fin.root_s)) <= MM);
    std::vector<Local> local = toLocal(part, spine, fin.root_s);
    for (Local v : local)
        CHECK(side * v.left >= -MM);

    auto root = std::max_element(
        local.begin(), local.end(), [&](Local a, Local b) {
            bool a_root = std::abs(a.back) <= MM, b_root = std::abs(b.back) <= MM;
            if (a_root != b_root)
                return !a_root;
            return side * a.left < side * b.left;
        });
    REQUIRE(std::abs(root->back) <= MM);
    float root_lat = side * root->left;
    CHECK(root_lat >= fin.root_half_width_min * L - MM);
    CHECK(root_lat <= fin.root_half_width_max * L + MM);

    auto tip = std::max_element(local.begin(), local.end(),
                                [](Local a, Local b) { return a.back < b.back; });
    CHECK(std::abs(tip->back - (fin.tip_s - fin.root_s) * L) <= MM);
    CHECK(std::abs(side * tip->left - (root_lat + fin.span * L)) <= MM);

    bool has_root_end = std::any_of(local.begin(), local.end(), [&](Local v) {
        return std::abs(v.back - (fin.root_end_s - fin.root_s) * L) <= MM &&
               side * v.left > MM;
    });
    CHECK(has_root_end);
}

} // namespace

TEST_CASE("R15: cephalofoil attached at s = 0 and oriented by the tangent") {
    for (const KnownSpine &spine : testSpines()) {
        CAPTURE(spine.nodes.size());
        CAPTURE(spine.radius);
        CAPTURE(spine.bend);
        BodyShape shape = buildBodyShape(spine.nodes, L);
        REQUIRE(shape.rigid_parts.size() == BodyShape::RIGID_PART_COUNT);
        const std::vector<Vector2> &head =
            shape.rigid_parts[BodyShape::CEPHALOFOIL];
        REQUIRE(head.size() >= 3);
        CHECK(dist(head[0], spine.at(0.0f)) <= MM);

        std::vector<Local> local = toLocal(head, spine, 0.0f);
        auto [min_back, max_back] = std::minmax_element(
            local.begin(), local.end(),
            [](Local a, Local b) { return a.back < b.back; });
        auto [min_left, max_left] = std::minmax_element(
            local.begin(), local.end(),
            [](Local a, Local b) { return a.left < b.left; });
        CHECK(std::abs(min_back->back) <= MM);
        CHECK(std::abs(max_back->back - 0.08f * L) <= MM);
        CHECK(std::abs(max_left->left - 0.135f * L) <= MM);
        CHECK(std::abs(min_left->left + 0.135f * L) <= MM);
        CHECK(hasVertex(local, 0.06f * L, 0.0f));
    }
}

TEST_CASE("R15: pectoral fins attached at s = 0.26 and oriented by the "
          "tangent") {
    for (const KnownSpine &spine : testSpines()) {
        CAPTURE(spine.nodes.size());
        CAPTURE(spine.radius);
        CAPTURE(spine.bend);
        BodyShape shape = buildBodyShape(spine.nodes, L);
        REQUIRE(shape.rigid_parts.size() == BodyShape::RIGID_PART_COUNT);
        checkFin(shape.rigid_parts[BodyShape::PECTORAL_LEFT], spine, PECTORAL,
                 LEFT);
        checkFin(shape.rigid_parts[BodyShape::PECTORAL_RIGHT], spine, PECTORAL,
                 RIGHT);
    }
}

TEST_CASE("R15: pelvic fins attached at s = 0.54 and oriented by the "
          "tangent") {
    for (const KnownSpine &spine : testSpines()) {
        CAPTURE(spine.nodes.size());
        CAPTURE(spine.radius);
        CAPTURE(spine.bend);
        BodyShape shape = buildBodyShape(spine.nodes, L);
        REQUIRE(shape.rigid_parts.size() == BodyShape::RIGID_PART_COUNT);
        checkFin(shape.rigid_parts[BodyShape::PELVIC_LEFT], spine, PELVIC,
                 LEFT);
        checkFin(shape.rigid_parts[BodyShape::PELVIC_RIGHT], spine, PELVIC,
                 RIGHT);
    }
}

TEST_CASE("R15: only the cephalofoil and the paired fins, no dorsal fins") {
    for (const KnownSpine &spine : testSpines()) {
        CAPTURE(spine.nodes.size());
        CAPTURE(spine.radius);
        CAPTURE(spine.bend);
        BodyShape shape = buildBodyShape(spine.nodes, L);
        CHECK(shape.rigid_parts.size() == 5);
    }
}

namespace {

float cross(Vector2 o, Vector2 a, Vector2 b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

} // namespace

TEST_CASE("R17: every rigid-part fan triangle is wound like the outline "
          "strip, so the draw culls none of them") {
    for (const KnownSpine &spine : testSpines()) {
        CAPTURE(spine.nodes.size());
        CAPTURE(spine.radius);
        CAPTURE(spine.bend);
        BodyShape shape = buildBodyShape(spine.nodes, L);
        float strip_sign =
            cross(shape.right[0], shape.left[0], shape.right[1]) > 0 ? 1 : -1;
        for (size_t p = 0; p < shape.rigid_parts.size(); ++p) {
            CAPTURE(p);
            const std::vector<Vector2> &part = shape.rigid_parts[p];
            for (size_t i = 1; i + 1 < part.size(); ++i) {
                CAPTURE(i);
                CHECK(strip_sign * cross(part[0], part[i], part[i + 1]) > 0);
            }
        }
    }
}
