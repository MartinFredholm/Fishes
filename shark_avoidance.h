#pragma once

#include "kinematics.h"
#include "motion.h"
#include <vector>

constexpr float SHARK_AVOID_HORIZON = 2.5f;
constexpr float SHARK_AVOID_MARGIN = 0.2f;
constexpr float SHARK_CONTACT_DISTANCE = 0.36f;
constexpr float SHARK_AVOID_DT = 1.0f / 30.0f;

struct OtherShark {
    Kinematics kin;
    std::vector<Vector2> body_line;
};

// World frame, meters, y down. Positive turn rate turns screen-left of the
// swimming direction. others does not contain the shark itself.
Commands avoidSharks(const Kinematics &self, Commands commanded,
                     const std::vector<OtherShark> &others,
                     const MotionParams &params);
