#pragma once

#include "kinematics.h"
#include <vector>

constexpr int BODY_LINE_POINTS = 5;

struct BodyParams {
    float length;
    int node_count;
    float wave_amplitude;
    float wave_number;
    float strouhal;
    int length_passes;
    float front_stiff_s;
    // In rad/s.
    float glide_full_yaw_rate;
    // In s.
    float glide_time_constant;
};

// arc increases toward the nose.
struct PathPoint {
    Vector2 pos;
    float arc;
};

struct SharkBody {
    std::vector<PathPoint> path;
    float phase = 0.0f;
    float wave_scale = 1.0f;
    std::vector<Vector2> spine;
    std::vector<Vector2> body_line;
};

SharkBody seedSharkBody(const Kinematics &kin, const BodyParams &params);

SharkBody updateSharkBody(const SharkBody &body, const Kinematics &kin,
                          const BodyParams &params, float dt);

Vector2 nearestPointOnBodyLine(Vector2 p, const std::vector<Vector2> &body_line);

// left is screen-left of the swimming direction (y down). Each rigid part is a
// triangle fan around its first vertex, the attachment point on the spine,
// wound like the outline strip (right, left, next right).
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
