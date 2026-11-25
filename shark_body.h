#pragma once

#include "kinematics.h"
#include <vector>

struct BodyParams {
    float length;
    int node_count;
    float wave_amplitude;
    float wave_number;
    float strouhal;
    int length_passes;
    float pivot;
};

struct SharkBody {
    float phase = 0.0f;
    std::vector<Vector2> pivot_path;
    std::vector<Vector2> spine;
};

SharkBody seedSharkBody(const Kinematics &kin, const BodyParams &params);

SharkBody updateSharkBody(const SharkBody &body, const Kinematics &kin,
                          const BodyParams &params, float dt);
