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
};

// arc is the distance travelled along the path, in meters; it increases toward
// the nose.
struct PathPoint {
    Vector2 pos;
    float arc;
};

struct SharkBody {
    std::vector<PathPoint> path;
    float phase = 0.0f;
    std::vector<Vector2> spine;
};

SharkBody seedSharkBody(const Kinematics &kin, const BodyParams &params);

SharkBody updateSharkBody(const SharkBody &body, const Kinematics &kin,
                          const BodyParams &params, float dt);
