#pragma once

#include "motion.h"
#include "shark_body.h"
#include <cmath>
#include <vector>

inline constexpr MotionParams SHARK_PARAMS = {.v_min = 0.5f,
                                              .v_cruise = 1.0f,
                                              .v_max = 2.5f,
                                              .r_max = 0.8f,
                                              .T_v = 1.0f,
                                              .T_r = 0.4f,
                                              .K_phi = 4.0f,
                                              .tau = 0.3f,
                                              .wall_clearance = 0.6f};

inline constexpr float L = 3.0f;
inline constexpr int NODES = 24;
inline constexpr float MM = 0.001f;
inline constexpr float TIGHTEST_TURN_RADIUS = 0.625f;
inline constexpr double PI_D = 3.14159265358979323846;

inline BodyParams bodyParams(float wave_amplitude, int node_count = NODES,
                             int length_passes = 1) {
    return {.length = L,
            .node_count = node_count,
            .wave_amplitude = wave_amplitude,
            .wave_number = 5.3f,
            .strouhal = 0.3f,
            .length_passes = length_passes};
}

inline Kinematics kinAt(Vector2 pos, float phi, float v, float r = 0.0f) {
    return {pos, phi, v, r, {std::cos(phi), -std::sin(phi)}};
}

inline Kinematics turnStep(const Kinematics &kin, float r, float dt) {
    Vector2 pos = {kin.pos.x + kin.heading.x * kin.v * dt,
                   kin.pos.y + kin.heading.y * kin.v * dt};
    return kinAt(pos, kin.phi + r * dt, kin.v, r);
}

inline Kinematics straightStep(const Kinematics &kin, float dt) {
    return turnStep(kin, 0.0f, dt);
}

inline float dist(Vector2 a, Vector2 b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

inline float cross(Vector2 o, Vector2 a, Vector2 b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

inline std::vector<Vector2> straightSpine(Vector2 nose, Vector2 fwd,
                                          int nodes = NODES) {
    std::vector<Vector2> spine(nodes);
    for (int i = 0; i < nodes; ++i) {
        float back = L * i / (nodes - 1);
        spine[i] = {nose.x - fwd.x * back, nose.y - fwd.y * back};
    }
    return spine;
}

// bend = +1 curves toward screen-left (y down).
inline std::vector<Vector2> arcSpine(Vector2 centre, float radius, float bend,
                                     int nodes = NODES) {
    std::vector<Vector2> spine(nodes);
    for (int i = 0; i < nodes; ++i) {
        float theta = bend * (L * i / (nodes - 1)) / radius;
        spine[i] = {centre.x + radius * std::cos(theta),
                    centre.y + radius * std::sin(theta)};
    }
    return spine;
}
