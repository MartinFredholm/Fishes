#include "doctest/doctest.h"

#include "shark_avoidance.h"
#include "test_support.h"
#include <cmath>
#include <vector>

namespace {

constexpr float HALF_PI = 1.57079632679f;
constexpr float PI_F = 3.14159265359f;

constexpr MotionParams SHARK_PARAMS = {.v_min = 0.5f,
                                       .v_cruise = 1.0f,
                                       .v_max = 2.5f,
                                       .r_max = 0.8f,
                                       .T_v = 1.0f,
                                       .T_r = 0.4f,
                                       .K_phi = 4.0f,
                                       .tau = 0.3f,
                                       .wall_clearance = 0.6f};

constexpr Commands STRAIGHT_AT_CRUISE = {.v_cmd = 1.0f, .r_cmd = 0.0f};
constexpr Commands ODD_COMMANDS = {.v_cmd = 1.3f, .r_cmd = 0.2f};

OtherShark straightShark(Vector2 nose, float phi, float v) {
    Kinematics kin = kinAt(nose, phi, v);
    std::vector<Vector2> body_line(BODY_LINE_POINTS);
    for (int i = 0; i < BODY_LINE_POINTS; ++i) {
        float back = L * i / (BODY_LINE_POINTS - 1);
        body_line[i] = {nose.x - kin.heading.x * back,
                        nose.y - kin.heading.y * back};
    }
    return {kin, body_line};
}

void checkUnchanged(Commands result, Commands commanded) {
    CHECK(result.v_cmd == commanded.v_cmd);
    CHECK(result.r_cmd == commanded.r_cmd);
}

const Kinematics SELF_EAST_MID_WORLD = kinAt({10.0f, 8.0f}, 0.0f, 1.0f);

} // namespace

TEST_SUITE("shark avoidance") {

TEST_CASE("with no other shark the commands come back unchanged") {
    Commands result =
        avoidSharks(SELF_EAST_MID_WORLD, ODD_COMMANDS, {}, SHARK_PARAMS);
    checkUnchanged(result, ODD_COMMANDS);
}

TEST_CASE("another shark far beyond reach within the horizon leaves the "
          "commands unchanged") {
    std::vector<OtherShark> others = {
        straightShark({25.0f, 12.0f}, 0.0f, 1.0f)};
    Commands result =
        avoidSharks(kinAt({5.0f, 4.0f}, 0.0f, 1.0f), ODD_COMMANDS, others,
                    SHARK_PARAMS);
    checkUnchanged(result, ODD_COMMANDS);
}

TEST_CASE("head-on, another shark ahead on the same line swimming toward "
          "the nose: takeover at v_min with a full turn") {
    std::vector<OtherShark> others = {straightShark({14.0f, 8.0f}, PI_F, 1.0f)};
    Commands result = avoidSharks(SELF_EAST_MID_WORLD, STRAIGHT_AT_CRUISE,
                                  others, SHARK_PARAMS);
    CHECK(result.v_cmd == SHARK_PARAMS.v_min);
    CHECK(std::abs(result.r_cmd) == SHARK_PARAMS.r_max);
}

TEST_CASE("a body line ahead and to the right gives a left turn") {
    std::vector<OtherShark> others = {
        straightShark({11.0f, 8.3f}, HALF_PI, 0.0f)};
    Commands result = avoidSharks(SELF_EAST_MID_WORLD, STRAIGHT_AT_CRUISE,
                                  others, SHARK_PARAMS);
    CHECK(result.r_cmd == SHARK_PARAMS.r_max);
}

TEST_CASE("a body line ahead and to the left gives a right turn") {
    std::vector<OtherShark> others = {
        straightShark({11.0f, 7.7f}, -HALF_PI, 0.0f)};
    Commands result = avoidSharks(SELF_EAST_MID_WORLD, STRAIGHT_AT_CRUISE,
                                  others, SHARK_PARAMS);
    CHECK(result.r_cmd == -SHARK_PARAMS.r_max);
}

TEST_CASE("a shark crossing the predicted path within the horizon causes a "
          "takeover, the same shark held still does not") {
    Vector2 nose_now = {12.0f, 10.5f};
    REQUIRE(dist(nose_now, SELF_EAST_MID_WORLD.pos) > 3.0f);

    SUBCASE("crossing") {
        std::vector<OtherShark> others = {
            straightShark(nose_now, HALF_PI, 1.25f)};
        Commands result = avoidSharks(SELF_EAST_MID_WORLD, STRAIGHT_AT_CRUISE,
                                      others, SHARK_PARAMS);
        CHECK(result.v_cmd == SHARK_PARAMS.v_min);
    }
    SUBCASE("held still") {
        std::vector<OtherShark> others = {
            straightShark(nose_now, HALF_PI, 0.0f)};
        Commands result = avoidSharks(SELF_EAST_MID_WORLD, STRAIGHT_AT_CRUISE,
                                      others, SHARK_PARAMS);
        checkUnchanged(result, STRAIGHT_AT_CRUISE);
    }
}

TEST_CASE("side by side, parallel at the same heading and speed, outside "
          "contact distance plus margin: no takeover") {
    constexpr float GAP_BEYOND_TAKEOVER = 0.04f;
    constexpr float BEYOND_CONTACT_PLUS_MARGIN =
        SHARK_CONTACT_DISTANCE + SHARK_AVOID_MARGIN + GAP_BEYOND_TAKEOVER;
    std::vector<OtherShark> others = {straightShark(
        {10.0f, 8.0f + BEYOND_CONTACT_PLUS_MARGIN}, 0.0f, 1.0f)};
    Commands result = avoidSharks(SELF_EAST_MID_WORLD, STRAIGHT_AT_CRUISE,
                                  others, SHARK_PARAMS);
    checkUnchanged(result, STRAIGHT_AT_CRUISE);
}

TEST_CASE("wall trap, a threat ahead and a wall close on the left: the step "
          "does not pick left") {
    constexpr float TOP_WALL_GAP = 1.6f;
    Kinematics self = kinAt({10.0f, TOP_WALL_GAP}, 0.0f, 1.0f);
    std::vector<OtherShark> others = {
        straightShark({14.0f, TOP_WALL_GAP}, PI_F, 1.0f)};
    Commands result =
        avoidSharks(self, STRAIGHT_AT_CRUISE, others, SHARK_PARAMS);
    CHECK(result.v_cmd == SHARK_PARAMS.v_min);
    CHECK(result.r_cmd != SHARK_PARAMS.r_max);
}

TEST_CASE("symmetric head-on with the shark already turning right: it picks "
          "right, even with left ahead by less than the tie") {
    constexpr float SLIGHTLY_TURNING_RIGHT = -0.001f;
    constexpr float OFFSET_RIGHT_FAVOURING_LEFT_WITHIN_TIE = 0.002f;
    Kinematics self = kinAt({10.0f, 8.0f}, 0.0f, 1.0f, SLIGHTLY_TURNING_RIGHT);
    std::vector<OtherShark> others = {straightShark(
        {14.0f, 8.0f + OFFSET_RIGHT_FAVOURING_LEFT_WITHIN_TIE}, PI_F, 1.0f)};
    Commands result =
        avoidSharks(self, STRAIGHT_AT_CRUISE, others, SHARK_PARAMS);
    CHECK(result.v_cmd == SHARK_PARAMS.v_min);
    CHECK(result.r_cmd == -SHARK_PARAMS.r_max);
}

}

TEST_SUITE("shark avoidance, reviewer") {

TEST_CASE("side by side inside contact distance plus margin but outside "
          "contact distance: takeover") {
    constexpr float BETWEEN_CONTACT_AND_CONTACT_PLUS_MARGIN = 0.5f;
    std::vector<OtherShark> others = {straightShark(
        {10.0f, 8.0f + BETWEEN_CONTACT_AND_CONTACT_PLUS_MARGIN}, 0.0f, 1.0f)};
    Commands result = avoidSharks(SELF_EAST_MID_WORLD, STRAIGHT_AT_CRUISE,
                                  others, SHARK_PARAMS);
    CHECK(result.v_cmd == SHARK_PARAMS.v_min);
}

TEST_CASE("the trigger uses the commanded path, not the current heading") {
    constexpr Commands FULL_LEFT_AT_CRUISE = {.v_cmd = 1.0f, .r_cmd = 0.8f};
    Vector2 top_of_left_turn_circle = {10.0f, 5.5f};
    REQUIRE(SELF_EAST_MID_WORLD.pos.y - top_of_left_turn_circle.y >
            SHARK_CONTACT_DISTANCE + SHARK_AVOID_MARGIN);
    std::vector<OtherShark> others = {
        straightShark(top_of_left_turn_circle, -HALF_PI, 0.0f)};

    SUBCASE("commanded straight: unchanged") {
        Commands result = avoidSharks(SELF_EAST_MID_WORLD, STRAIGHT_AT_CRUISE,
                                      others, SHARK_PARAMS);
        checkUnchanged(result, STRAIGHT_AT_CRUISE);
    }
    SUBCASE("commanded full left into the line: takeover") {
        Commands result = avoidSharks(SELF_EAST_MID_WORLD, FULL_LEFT_AT_CRUISE,
                                      others, SHARK_PARAMS);
        CHECK(result.v_cmd == SHARK_PARAMS.v_min);
    }
}

TEST_CASE("wall trap mirrored, a wall close on the right: the step does not "
          "pick right") {
    constexpr float BOTTOM_WALL_GAP = 1.6f;
    float y = WORLD_HEIGHT - BOTTOM_WALL_GAP;
    Kinematics self = kinAt({10.0f, y}, 0.0f, 1.0f, -0.001f);
    std::vector<OtherShark> others = {straightShark({14.0f, y}, PI_F, 1.0f)};
    Commands result =
        avoidSharks(self, STRAIGHT_AT_CRUISE, others, SHARK_PARAMS);
    CHECK(result.v_cmd == SHARK_PARAMS.v_min);
    CHECK(result.r_cmd != -SHARK_PARAMS.r_max);
}

TEST_CASE("head-on offset clearly to the right while turning right: the "
          "better side wins over the current turn") {
    constexpr float OFFSET_RIGHT_BEYOND_TIE = 0.15f;
    Kinematics self = kinAt({10.0f, 8.0f}, 0.0f, 1.0f, -0.001f);
    std::vector<OtherShark> others = {straightShark(
        {14.0f, 8.0f + OFFSET_RIGHT_BEYOND_TIE}, PI_F, 1.0f)};
    Commands result =
        avoidSharks(self, STRAIGHT_AT_CRUISE, others, SHARK_PARAMS);
    CHECK(result.v_cmd == SHARK_PARAMS.v_min);
    CHECK(result.r_cmd == SHARK_PARAMS.r_max);
}

TEST_CASE("symmetric head-on turning left: it picks left") {
    Kinematics self = kinAt({10.0f, 8.0f}, 0.0f, 1.0f, 0.001f);
    std::vector<OtherShark> others = {straightShark({14.0f, 8.0f}, PI_F, 1.0f)};
    Commands result =
        avoidSharks(self, STRAIGHT_AT_CRUISE, others, SHARK_PARAMS);
    CHECK(result.r_cmd == SHARK_PARAMS.r_max);
}

}
