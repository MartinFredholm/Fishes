#include "raylib.h"
#include "raymath.h"
#include <cmath>
#include <stdio.h>

using namespace std;

#define FISH_WIDTH 20
#define FISH_LENGTH 30
#define FISH_COLOR BLUE
#define FISH_SPEED 50

struct Kinematics {
    Vector2 pos;
    float phi;
    float v;
    float r;
};

struct MotionParams {
    float v_min;
    float v_max;
    float r_max;
};

struct Fish {
    Kinematics kin;
};

Kinematics integrate(const Kinematics& kin, float v_cmd, float r_cmd,
                     const MotionParams& params, float dt);

void DrawFish(const Fish& fish);

int main() {
    InitWindow(800, 450, "raylib example -- basic window");
    SetTargetFPS(60);
    float dt = 0.0f;
    Fish fish = Fish({{0.0, 100.0}, -PI / 4, FISH_SPEED, 0});
    while (!WindowShouldClose()) {
        dt = GetFrameTime();
        Vector2 heading = {cos(fish.kin.phi), -sin(fish.kin.phi)};
        fish.kin.pos += heading * fish.kin.v * dt;
        BeginDrawing();
        ClearBackground(BLACK);
        DrawFish(fish);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}

void DrawFish(const Fish& fish) {
    Vector2 v_back_1 = {fish.kin.pos.x - cos(fish.kin.phi) * FISH_LENGTH / 2 -
                            sin(fish.kin.phi) * FISH_WIDTH / 2,
                        fish.kin.pos.y + sin(fish.kin.phi) * FISH_LENGTH / 2 -
                            cos(fish.kin.phi) * FISH_WIDTH / 2};

    Vector2 v_back_2 = {fish.kin.pos.x - cos(fish.kin.phi) * FISH_LENGTH / 2 +
                            sin(fish.kin.phi) * FISH_WIDTH / 2,
                        fish.kin.pos.y + sin(fish.kin.phi) * FISH_LENGTH / 2 +
                            cos(fish.kin.phi) * FISH_WIDTH / 2};

    Vector2 v_front = {fish.kin.pos.x + cos(fish.kin.phi) * FISH_LENGTH / 2,
                       fish.kin.pos.y - sin(fish.kin.phi) * FISH_LENGTH / 2};

    DrawTriangle(v_back_1, v_back_2, v_front, FISH_COLOR);
}
