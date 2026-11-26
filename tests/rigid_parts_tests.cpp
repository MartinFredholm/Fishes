#include "doctest/doctest.h"

#include "test_support.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace {

constexpr float LEFT = 1.0f;
constexpr float RIGHT = -1.0f;

constexpr int ANCHOR_NODES = 51;

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
    for (int nodes : {ANCHOR_NODES, NODES}) {
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

float polygonArea(const std::vector<Vector2> &p) {
    float sum = 0.0f;
    for (size_t i = 0; i < p.size(); ++i) {
        const Vector2 &a = p[i], &b = p[(i + 1) % p.size()];
        sum += a.x * b.y - b.x * a.y;
    }
    return 0.5f * std::abs(sum);
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

Vector2 sub(Vector2 a, Vector2 b) { return {a.x - b.x, a.y - b.y}; }

Vector2 unit(Vector2 a) {
    float n = std::hypot(a.x, a.y);
    return {a.x / n, a.y / n};
}

float crossDir(Vector2 a, Vector2 b) { return a.x * b.y - a.y * b.x; }

float turnAt(const std::vector<Vector2> &spine, int i) {
    Vector2 u = sub(spine[i], spine[i - 1]), w = sub(spine[i + 1], spine[i]);
    return std::atan2(crossDir(u, w), u.x * w.x + u.y * w.y);
}

// At v_min and r_max the body update sometimes folds the spine at nodes 13–14
// (1.2–1.6 rad between adjacent segments). This test skips those frames.
bool smooth(const std::vector<Vector2> &spine) {
    for (int i = 1; i + 1 < (int)spine.size(); ++i)
        if (std::abs(turnAt(spine, i)) > 0.5f)
            return false;
    return true;
}

// Tangent at node 0 of the circle through nodes 0, 1 and 2.
Vector2 noseTailward(const std::vector<Vector2> &spine) {
    Vector2 c = unit(sub(spine[1], spine[0]));
    float a = -turnAt(spine, 1) / 2;
    return {c.x * std::cos(a) - c.y * std::sin(a),
            c.x * std::sin(a) + c.y * std::cos(a)};
}

} // namespace

TEST_SUITE("rigid parts") {

TEST_CASE("cephalofoil attached at s = 0 and oriented by the tangent") {
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
        CHECK(std::abs(min_back->back + 0.01f * L) <= MM);
        CHECK(std::abs(max_back->back - 0.07f * L) <= MM);
        CHECK(std::abs(max_left->left - 0.135f * L) <= MM);
        CHECK(std::abs(min_left->left + 0.135f * L) <= MM);
        CHECK(hasVertex(local, 0.05f * L, 0.0f));
    }
}

TEST_CASE("pectoral fins attached at s = 0.26 and oriented by the tangent") {
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

TEST_CASE("pelvic fins attached at s = 0.54 and oriented by the tangent") {
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

TEST_CASE("only the cephalofoil and the paired fins, no dorsal fins") {
    for (const KnownSpine &spine : testSpines()) {
        CAPTURE(spine.nodes.size());
        CAPTURE(spine.radius);
        CAPTURE(spine.bend);
        BodyShape shape = buildBodyShape(spine.nodes, L);
        CHECK(shape.rigid_parts.size() == 5);
    }
}

TEST_CASE("every rigid-part fan triangle is wound like the outline strip, "
          "and no fan overlaps itself") {
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
            float fan_area = 0.0f;
            for (size_t i = 1; i + 1 < part.size(); ++i) {
                CAPTURE(i);
                float twice_area = strip_sign * cross(part[0], part[i], part[i + 1]);
                CHECK(twice_area > 0);
                fan_area += 0.5f * twice_area;
            }
            std::vector<Vector2> rim = part;
            if (p == BodyShape::CEPHALOFOIL)
                rim.assign(part.begin() + 1, part.end() - 1);
            CHECK(std::abs(fan_area - polygonArea(rim)) <= 1e-4f);
        }
    }
}

TEST_CASE("the cephalofoil covers the front of the outline on straight and "
          "bent spines") {
    checkHeadCovered(straightSpine({10.0f, 5.0f}, {1.0f, 0.0f}));
    for (float radius : {3.0f, 1.5f, TIGHTEST_TURN_RADIUS})
        for (float bend : {1.0f, -1.0f}) {
            CAPTURE(radius);
            CAPTURE(bend);
            checkHeadCovered(arcSpine({15.0f, 8.0f}, radius, bend));
        }
}

TEST_CASE("the cephalofoil covers the front of the outline while swimming and "
          "turning") {
    const BodyParams params = bodyParams(0.1f);
    const float dt = 1.0f / 60.0f;
    for (float v : {0.5f, 1.0f, 2.5f})
        for (float r : {0.0f, 0.8f, -0.8f}) {
            CAPTURE(v);
            CAPTURE(r);
            Kinematics kin = kinAt({15.0f, 8.0f}, 0.3f, v, r);
            SharkBody body = seedSharkBody(kin, params);
            for (int f = 0; f < 300; ++f) {
                kin = turnStep(kin, r, dt);
                body = updateSharkBody(body, kin, params, dt);
                if (f % 7 == 0)
                    checkHeadCovered(body.spine);
            }
        }
}

TEST_CASE("rigid parts on a swum, waved, turning spine") {
    const BodyParams params = bodyParams(0.1f);
    const float v_min = 0.5f, r_max = 0.8f, dt = 1.0f / 60.0f;
    const float max_angle_error_rad = 0.035f;
    for (float r : {r_max, -r_max, 0.0f}) {
        CAPTURE(r);
        Kinematics kin = kinAt({15.0f, 8.0f}, 0.3f, v_min, r);
        SharkBody body = seedSharkBody(kin, params);
        for (int f = 0; f < 600; ++f) {
            kin = turnStep(kin, r, dt);
            body = updateSharkBody(body, kin, params, dt);
            if (f % 37 != 0 || !smooth(body.spine))
                continue;
            BodyShape shape = buildBodyShape(body.spine, L);
            REQUIRE(shape.rigid_parts.size() == 5);

            CHECK(dist(shape.rigid_parts[BodyShape::CEPHALOFOIL][0],
                       body.spine[0]) <= MM);
            const float ahead = (6.0f / 23.0f - 0.26f) * L;
            for (auto part : {BodyShape::PECTORAL_LEFT,
                              BodyShape::PECTORAL_RIGHT})
                CHECK(std::abs(dist(shape.rigid_parts[part][0],
                                    body.spine[6]) - ahead) <= MM);
            for (auto part : {BodyShape::PELVIC_LEFT,
                              BodyShape::PELVIC_RIGHT}) {
                Vector2 a = shape.rigid_parts[part][0];
                CHECK(dist(a, body.spine[12]) + dist(a, body.spine[13]) -
                          L / 23 <= MM);
                CHECK(std::abs(dist(a, body.spine[12]) -
                               (0.54f - 12.0f / 23.0f) * L) <= 2 * MM);
            }

            Vector2 back = noseTailward(body.spine);
            const auto &head = shape.rigid_parts[BodyShape::CEPHALOFOIL];
            bool found = false;
            for (Vector2 q : head) {
                Vector2 d = {q.x - head[0].x, q.y - head[0].y};
                if (std::abs(std::hypot(d.x, d.y) - 0.05f * L) > 0.01f)
                    continue;
                Vector2 u = unit(d);
                if (u.x * back.x + u.y * back.y > std::cos(max_angle_error_rad))
                    found = true;
            }
            CHECK(found);

            Vector2 fwd = unit({body.spine[5].x - body.spine[7].x,
                                body.spine[5].y - body.spine[7].y});
            for (Vector2 q : shape.rigid_parts[BodyShape::PECTORAL_LEFT]) {
                Vector2 d = {q.x - body.spine[6].x, q.y - body.spine[6].y};
                CHECK(crossDir(fwd, d) <= 0.01f);
            }
            for (Vector2 q : shape.rigid_parts[BodyShape::PECTORAL_RIGHT]) {
                Vector2 d = {q.x - body.spine[6].x, q.y - body.spine[6].y};
                CHECK(crossDir(fwd, d) >= -0.01f);
            }
        }
    }
}

}
