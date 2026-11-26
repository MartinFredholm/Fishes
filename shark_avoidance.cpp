#include "shark_avoidance.h"
#include "raymath.h"
#include "shark_body.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {

struct Clearance {
    float shark;
    float wall;
    float score() const { return std::min(shark, wall); }
};

float sharkClearance(Vector2 nose, float t,
                     const std::vector<OtherShark> &others) {
    float clearance = std::numeric_limits<float>::infinity();
    for (const OtherShark &other : others) {
        Vector2 nose_in_line_frame =
            nose - other.kin.heading * (other.kin.v * t);
        Vector2 nearest =
            nearestPointOnBodyLine(nose_in_line_frame, other.body_line);
        clearance = std::min(clearance,
                             Vector2Distance(nose_in_line_frame, nearest) -
                                 SHARK_CONTACT_DISTANCE);
    }
    return clearance;
}

float wallClearance(Vector2 nose, float wall_clearance) {
    return std::min({nose.x, WORLD_WIDTH - nose.x, nose.y,
                     WORLD_HEIGHT - nose.y}) -
           wall_clearance;
}

Clearance predictClearance(const Kinematics &self, Commands commands,
                           const std::vector<OtherShark> &others,
                           const MotionParams &params) {
    int steps = (int)std::lround(SHARK_AVOID_HORIZON / SHARK_AVOID_DT);
    Clearance clearance = {std::numeric_limits<float>::infinity(),
                           std::numeric_limits<float>::infinity()};
    Kinematics kin = self;
    for (int k = 1; k <= steps; ++k) {
        kin = integrate(kin, commands, params, SHARK_AVOID_DT);
        float t = k * SHARK_AVOID_DT;
        clearance.shark =
            std::min(clearance.shark, sharkClearance(kin.pos, t, others));
        clearance.wall = std::min(
            clearance.wall, wallClearance(kin.pos, params.wall_clearance));
    }
    return clearance;
}

} // namespace

Commands avoidSharks(const Kinematics &self, Commands commanded,
                     const std::vector<OtherShark> &others,
                     const MotionParams &params) {
    if (predictClearance(self, commanded, others, params).shark >=
        SHARK_AVOID_MARGIN)
        return commanded;

    const Commands left = {params.v_min, params.r_max};
    const Commands straight = {params.v_min, 0.0f};
    const Commands right = {params.v_min, -params.r_max};
    auto score = [&](Commands option) {
        return predictClearance(self, option, others, params).score();
    };
    float left_score = score(left);
    float right_score = score(right);

    bool turning_left = self.r >= 0.0f;
    bool tie = std::abs(left_score - right_score) < CLEARANCE_TIE;
    bool pick_left = tie ? turning_left : left_score > right_score;
    Commands side = pick_left ? left : right;
    float side_score = pick_left ? left_score : right_score;

    return score(straight) > side_score ? straight : side;
}
