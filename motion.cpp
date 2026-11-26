#include "motion.h"
#include "raymath.h"
#include <cmath>

Kinematics integrate(const Kinematics &kin, Commands commands,
                     const MotionParams &params, float dt) {
    float v_cmd = commands.v_cmd;
    float r_cmd = commands.r_cmd;
    float v_dot = (v_cmd - kin.v) / params.T_v;
    float r_dot = (r_cmd - kin.r) / params.T_r;

    Vector2 pos = kin.pos + kin.v * kin.heading * dt;

    float phi = kin.phi + kin.r * dt;
    float v = kin.v + v_dot * dt;
    float r = kin.r + r_dot * dt;
    Vector2 heading = {std::cos(phi), -std::sin(phi)};
    return {pos, phi, v, r, heading};
}
