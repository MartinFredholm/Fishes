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

// World meters. left and right have one point per spine node; left is on the
// screen-left of the swimming direction (y down).
// Each rigid part is a polygon whose first vertex is its attachment point on
// the spine. It is a triangle fan around that vertex, wound like the
// outline's strip triangles (right, left, next right).
struct BodyShape {
    enum RigidPart {
        CEPHALOFOIL,
        PECTORAL_LEFT,
        PECTORAL_RIGHT,
        PELVIC_LEFT,
        PELVIC_RIGHT,
        RIGID_PART_COUNT
    };

    std::vector<Vector2> left;
    std::vector<Vector2> right;
    std::vector<std::vector<Vector2>> rigid_parts;
};

BodyShape buildBodyShape(const std::vector<Vector2> &spine, float length);
