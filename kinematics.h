#pragma once

#include "raylib.h"
#include <cmath>

struct Kinematics {
    Vector2 pos{};
    float phi = 0.0f;
    float v = 0.0f;
    float r = 0.0f;
    Vector2 heading = {std::cos(phi), -std::sin(phi)};
};
