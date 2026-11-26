#pragma once

#include "kinematics.h"

struct MotionParams {
    float v_min;
    float v_cruise;
    float v_max;
    float r_max;
    float T_v;
    float T_r;
    float K_phi;
    float tau;
    float wall_clearance;
};

struct Commands {
    float v_cmd;
    float r_cmd;
};

constexpr float METERS_PER_PIXEL = 0.02f;

constexpr int WINDOW_WIDTH = 1500;
constexpr int WINDOW_HEIGHT = 800;
constexpr float WORLD_WIDTH = WINDOW_WIDTH * METERS_PER_PIXEL;
constexpr float WORLD_HEIGHT = WINDOW_HEIGHT * METERS_PER_PIXEL;

constexpr float CLEARANCE_TIE = 0.01f;

Kinematics integrate(const Kinematics &kin, Commands commands,
                     const MotionParams &params, float dt);
