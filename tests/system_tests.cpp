#include "doctest/doctest.h"

#include "motion.h"
#include "shark_avoidance.h"
#include "test_support.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace {

constexpr Commands STRAIGHT_AT_CRUISE = {.v_cmd = 1.0f, .r_cmd = 0.0f};
constexpr float FRAME_DT = 1.0f / 60.0f;
constexpr float ENCOUNTER_SECONDS = 10.0f;
constexpr int PROBE_SAMPLES_PER_SEGMENT = 11;

struct SimShark {
    Kinematics kin;
    SharkBody body;
};

SimShark spawn(Vector2 nose, float phi) {
    Kinematics kin = kinAt(nose, phi, SHARK_PARAMS.v_cruise);
    return {kin, seedSharkBody(kin, bodyParams(0.1f))};
}

float bodyLineDistance(const std::vector<Vector2> &a,
                       const std::vector<Vector2> &b) {
    float best = INFINITY;
    for (size_t i = 1; i < a.size(); ++i)
        for (int k = 0; k < PROBE_SAMPLES_PER_SEGMENT; ++k) {
            float t = (float)k / (PROBE_SAMPLES_PER_SEGMENT - 1);
            Vector2 p = {a[i - 1].x + (a[i].x - a[i - 1].x) * t,
                         a[i - 1].y + (a[i].y - a[i - 1].y) * t};
            best = std::min(best, dist(p, nearestPointOnBodyLine(p, b)));
        }
    return best;
}

float closestApproachWithAvoidance(SimShark a, SimShark b) {
    float closest = bodyLineDistance(a.body.body_line, b.body.body_line);
    for (int k = 0; k < (int)(ENCOUNTER_SECONDS / FRAME_DT); ++k) {
        Commands cmd_a = avoidSharks(a.kin, STRAIGHT_AT_CRUISE,
                                     {{b.kin, b.body.body_line}}, SHARK_PARAMS);
        Commands cmd_b = avoidSharks(b.kin, STRAIGHT_AT_CRUISE,
                                     {{a.kin, a.body.body_line}}, SHARK_PARAMS);
        a.kin = integrate(a.kin, cmd_a, SHARK_PARAMS, FRAME_DT);
        b.kin = integrate(b.kin, cmd_b, SHARK_PARAMS, FRAME_DT);
        a.body = updateSharkBody(a.body, a.kin, bodyParams(0.1f), FRAME_DT);
        b.body = updateSharkBody(b.body, b.kin, bodyParams(0.1f), FRAME_DT);
        closest = std::min(closest, bodyLineDistance(a.body.body_line,
                                                     b.body.body_line));
    }
    return closest;
}

} // namespace

TEST_SUITE("shark encounters") {

TEST_CASE("positive turn rate through integrate bends the body line to "
          "screen-left") {
    SimShark s = spawn({15.0f, 8.0f}, 0.0f);
    constexpr Commands FULL_LEFT = {.v_cmd = 1.0f, .r_cmd = 0.8f};
    for (int k = 0; k < 120; ++k) {
        s.kin = integrate(s.kin, FULL_LEFT, SHARK_PARAMS, FRAME_DT);
        s.body = updateSharkBody(s.body, s.kin, bodyParams(0.1f), FRAME_DT);
    }
    CHECK(s.kin.pos.y < 8.0f);
    CHECK(cross(s.body.body_line[2], s.body.body_line[1],
                s.body.body_line[0]) < 0.0f);
}

TEST_CASE("head-on, both sharks running avoidance on real body lines: no "
          "shark contact") {
    CHECK(closestApproachWithAvoidance(spawn({9.0f, 8.0f}, 0.0f),
                                       spawn({21.0f, 8.0f}, PI_D)) >=
          SHARK_CONTACT_DISTANCE);
}

TEST_CASE("crossing at right angles, both running avoidance on real body "
          "lines: no shark contact") {
    CHECK(closestApproachWithAvoidance(spawn({10.0f, 8.0f}, 0.0f),
                                       spawn({15.0f, 13.0f}, PI_D / 2)) >=
          SHARK_CONTACT_DISTANCE);
}

TEST_CASE("one shark aimed at the other's tail half: no shark contact") {
    CHECK(closestApproachWithAvoidance(spawn({15.0f, 12.0f}, PI_D / 2),
                                       spawn({16.5f, 8.0f}, 0.0f)) >=
          SHARK_CONTACT_DISTANCE);
}

}
