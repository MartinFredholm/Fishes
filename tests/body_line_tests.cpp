#include "doctest/doctest.h"

#include "test_support.h"
#include <cmath>
#include <vector>

namespace {

constexpr float TOL_M = 1e-3f;
constexpr float BODY_LINE_S[] = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f};
const std::vector<float> UNEVEN_DTS = {1.0f / 30.0f, 0.003f, 1.0f / 90.0f,
                                       0.0f,         0.027f, 0.0001f,
                                       1.0f / 45.0f};
const std::vector<float> EVEN_DTS = {1.0f / 60.0f};

// y down: pos = c + R·(sin phi, cos phi) moves along heading
// (cos phi, −sin phi).
struct Circle {
    Vector2 c;
    float R;
    Kinematics at(float phi, float v) const {
        return kinAt({c.x + R * std::sin(phi), c.y + R * std::cos(phi)}, phi,
                     v);
    }
};

void checkBodyLineOnCircle(const SharkBody &body, const Circle &circle,
                           float nose_phi, float v) {
    REQUIRE(body.body_line.size() == BODY_LINE_POINTS);
    for (int i = 0; i < BODY_LINE_POINTS; ++i) {
        CAPTURE(i);
        float phi_at_s = nose_phi - BODY_LINE_S[i] * L / circle.R;
        CHECK(dist(body.body_line[i], circle.at(phi_at_s, v).pos) < TOL_M);
    }
}

void swimCircleAndCheck(const Circle &circle, float v,
                        const std::vector<float> &dts) {
    const BodyParams params = bodyParams(0.1f);
    float phi = 0.4f;
    Kinematics kin = circle.at(phi, v);
    SharkBody body = seedSharkBody(kin, params);
    float swum = 0.0f;
    for (int k = 0; swum < 3.0f * L; ++k) {
        float dt = dts[k % dts.size()];
        phi += v / circle.R * dt;
        swum += v * dt;
        kin = circle.at(phi, v);
        body = updateSharkBody(body, kin, params, dt);
        CHECK(dist(body.body_line[0], kin.pos) < TOL_M);
    }
    checkBodyLineOnCircle(body, circle, phi, v);
}

void swimStraightAndCheck(const std::vector<float> &dts) {
    const BodyParams params = bodyParams(0.1f);
    const float phi = -1.2f;
    Kinematics kin = kinAt({4.0f, 6.0f}, phi, 1.0f);
    const Vector2 start = kin.pos;
    SharkBody body = seedSharkBody(kin, params);
    for (int k = 0; k < 400; ++k) {
        float dt = dts[k % dts.size()];
        kin = straightStep(kin, dt);
        body = updateSharkBody(body, kin, params, dt);
    }
    REQUIRE(body.body_line.size() == BODY_LINE_POINTS);
    Vector2 fwd = {std::cos(phi), -std::sin(phi)};
    float nose_along =
        (kin.pos.x - start.x) * fwd.x + (kin.pos.y - start.y) * fwd.y;
    for (int i = 0; i < BODY_LINE_POINTS; ++i) {
        CAPTURE(i);
        Vector2 rel = {body.body_line[i].x - start.x,
                       body.body_line[i].y - start.y};
        float along = rel.x * fwd.x + rel.y * fwd.y;
        float across = rel.x * fwd.y - rel.y * fwd.x;
        CHECK(std::abs(across) < TOL_M);
        CHECK(std::abs((nose_along - along) - BODY_LINE_S[i] * L) < TOL_M);
    }
}

Vector2 pointAtArcBehindNose(const std::vector<Vector2> &trace,
                             double arc_behind_nose) {
    double walked = 0.0;
    for (size_t j = trace.size() - 1; j > 0; --j) {
        double seg = dist(trace[j], trace[j - 1]);
        if (walked + seg >= arc_behind_nose) {
            float t = seg > 0.0 ? (float)((arc_behind_nose - walked) / seg)
                                : 0.0f;
            return {trace[j].x + (trace[j - 1].x - trace[j].x) * t,
                    trace[j].y + (trace[j - 1].y - trace[j].y) * t};
        }
        walked += seg;
    }
    return trace.front();
}

} // namespace

TEST_SUITE("body line") {

TEST_CASE("body line has 5 points after seeding and after every update") {
    const BodyParams params = bodyParams(0.1f);
    Kinematics kin = kinAt({5.0f, 5.0f}, 0.4f, 1.0f);
    SharkBody body = seedSharkBody(kin, params);
    CHECK(body.body_line.size() == 5);
    for (int k = 0; k < 120; ++k) {
        kin = turnStep(kin, 0.6f, 1.0f / 60.0f);
        body = updateSharkBody(body, kin, params, 1.0f / 60.0f);
        CHECK(body.body_line.size() == 5);
    }
}

TEST_CASE("at spawn the body line is straight along the heading, nose to L "
          "behind") {
    const float phi = 0.7f;
    Kinematics kin = kinAt({10.0f, 5.0f}, phi, 1.0f);
    SharkBody body = seedSharkBody(kin, bodyParams(0.1f));
    REQUIRE(body.body_line.size() == BODY_LINE_POINTS);
    for (int i = 0; i < BODY_LINE_POINTS; ++i) {
        CAPTURE(i);
        Vector2 expected = {10.0f - std::cos(phi) * BODY_LINE_S[i] * L,
                            5.0f + std::sin(phi) * BODY_LINE_S[i] * L};
        CHECK(dist(body.body_line[i], expected) < TOL_M);
    }
}

TEST_CASE("straight path, even dt: body line on the path at s·L") {
    swimStraightAndCheck(EVEN_DTS);
}

TEST_CASE("straight path, uneven dt: body line on the path at s·L") {
    swimStraightAndCheck(UNEVEN_DTS);
}

TEST_CASE("cruise circle, even dt: body line on the circle at s·L") {
    swimCircleAndCheck({{8.0f, 6.0f}, 1.0f}, 1.0f, EVEN_DTS);
}

TEST_CASE("tightest circle, even dt: body line on the circle at s·L") {
    swimCircleAndCheck({{8.0f, 6.0f}, TIGHTEST_TURN_RADIUS}, 0.5f, EVEN_DTS);
}

TEST_CASE("tightest circle, uneven dt: body line on the circle at s·L") {
    swimCircleAndCheck({{8.0f, 6.0f}, TIGHTEST_TURN_RADIUS}, 0.5f, UNEVEN_DTS);
}

TEST_CASE("body line is the same with and without the body wave") {
    const BodyParams no_wave = bodyParams(0.0f);
    const BodyParams wave = bodyParams(0.1f);
    Kinematics kin = kinAt({15.0f, 8.0f}, 0.9f, 1.0f);
    SharkBody still = seedSharkBody(kin, no_wave);
    SharkBody waving = seedSharkBody(kin, wave);
    float t = 0.0f;
    for (int k = 0; k < 1000; ++k) {
        float dt = UNEVEN_DTS[k % UNEVEN_DTS.size()];
        t += dt;
        kin = turnStep(kin, 0.8f * std::sin(0.45f * t), dt);
        still = updateSharkBody(still, kin, no_wave, dt);
        waving = updateSharkBody(waving, kin, wave, dt);
        REQUIRE(still.body_line.size() == BODY_LINE_POINTS);
        REQUIRE(waving.body_line.size() == BODY_LINE_POINTS);
        for (int i = 0; i < BODY_LINE_POINTS; ++i) {
            CAPTURE(k);
            CAPTURE(i);
            CHECK(dist(still.body_line[i], waving.body_line[i]) <= 1e-6f);
        }
    }
}

TEST_CASE("varying tightest-radius turn, uneven dt: every update puts each "
          "point on the traced nose path at s·L behind the nose") {
    const BodyParams params = bodyParams(0.1f);
    const std::vector<float> dts = {1.0f / 30.0f, 0.004f, 1.0f / 75.0f, 0.0f,
                                    0.021f};
    const float v = 0.5f;
    const float r_max = v / TIGHTEST_TURN_RADIUS;
    Kinematics kin = kinAt({10.0f, 7.0f}, -0.3f, v);
    SharkBody body = seedSharkBody(kin, params);
    std::vector<Vector2> trace = {
        {kin.pos.x - kin.heading.x * L, kin.pos.y - kin.heading.y * L},
        kin.pos};
    float t = 0.0f;
    for (int k = 0; k < 1500; ++k) {
        float dt = dts[k % dts.size()];
        t += dt;
        kin = turnStep(kin, r_max * std::sin(0.7f * t), dt);
        trace.push_back(kin.pos);
        body = updateSharkBody(body, kin, params, dt);
        REQUIRE(body.body_line.size() == BODY_LINE_POINTS);
        for (int i = 0; i < BODY_LINE_POINTS; ++i) {
            CAPTURE(k);
            CAPTURE(i);
            double s = (double)i / (BODY_LINE_POINTS - 1);
            CHECK(dist(body.body_line[i],
                       pointAtArcBehindNose(trace, s * L)) < MM);
        }
    }
}

}

TEST_SUITE("nearest point on body line") {

constexpr float NEAREST_TOL_M = 1e-5f;

const std::vector<Vector2> STRAIGHT_LINE = {
    {4.0f, 2.0f}, {3.25f, 2.0f}, {2.5f, 2.0f}, {1.75f, 2.0f}, {1.0f, 2.0f}};

const std::vector<Vector2> RIGHT_ANGLE_LINE = {
    {0.0f, 0.0f}, {1.0f, 0.0f}, {2.0f, 0.0f}, {2.0f, 1.0f}, {2.0f, 2.0f}};

const std::vector<Vector2> HAIRPIN_LINE = {
    {0.0f, 0.0f}, {1.0f, 0.0f}, {2.0f, 0.0f}, {2.0f, 1.0f}, {0.0f, 1.0f}};

void checkNearest(Vector2 p, const std::vector<Vector2> &line,
                  Vector2 expected) {
    CHECK(dist(nearestPointOnBodyLine(p, line), expected) < NEAREST_TOL_M);
}

TEST_CASE("a position ahead of the nose gets the nose") {
    checkNearest({5.0f, 2.0f}, STRAIGHT_LINE, {4.0f, 2.0f});
    checkNearest({4.6f, 1.2f}, STRAIGHT_LINE, {4.0f, 2.0f});
}

TEST_CASE("a position behind the tail tip gets the tail tip") {
    checkNearest({0.2f, 2.0f}, STRAIGHT_LINE, {1.0f, 2.0f});
    checkNearest({0.5f, 3.1f}, STRAIGHT_LINE, {1.0f, 2.0f});
}

TEST_CASE("a position beside a segment gets its perpendicular foot") {
    checkNearest({3.6f, 1.3f}, STRAIGHT_LINE, {3.6f, 2.0f});
    checkNearest({2.9f, 2.4f}, STRAIGHT_LINE, {2.9f, 2.0f});
    checkNearest({2.1f, 0.5f}, STRAIGHT_LINE, {2.1f, 2.0f});
    checkNearest({1.3f, 2.8f}, STRAIGHT_LINE, {1.3f, 2.0f});
}

TEST_CASE("a position on the body line gets itself") {
    checkNearest({2.2f, 2.0f}, STRAIGHT_LINE, {2.2f, 2.0f});
}

TEST_CASE("inside a right-angle bend, the nearer of the two arms wins") {
    checkNearest({1.7f, 0.6f}, RIGHT_ANGLE_LINE, {2.0f, 0.6f});
    checkNearest({1.4f, 0.3f}, RIGHT_ANGLE_LINE, {1.4f, 0.0f});
}

TEST_CASE("inside a hairpin, the far segment wins over the segments at the "
          "nearest point") {
    checkNearest({1.0f, 0.6f}, HAIRPIN_LINE, {1.0f, 1.0f});
    checkNearest({0.5f, 0.3f}, HAIRPIN_LINE, {0.5f, 0.0f});
}

TEST_CASE("bent line: a position nearest to a segment interior far from the "
          "nearest vertex gets that segment's foot") {
    const std::vector<Vector2> zigzag = {
        {0.0f, 0.0f}, {3.0f, 0.0f}, {3.0f, 0.1f}, {0.0f, 0.1f}, {0.0f, 3.0f}};
    CHECK(dist(nearestPointOnBodyLine({1.5f, -0.5f}, zigzag), {1.5f, 0.0f}) <
          1e-5f);
    CHECK(dist(nearestPointOnBodyLine({1.5f, 0.06f}, zigzag), {1.5f, 0.1f}) <
          1e-5f);
    CHECK(dist(nearestPointOnBodyLine({-0.4f, 2.0f}, zigzag), {0.0f, 2.0f}) <
          1e-5f);
}

}
