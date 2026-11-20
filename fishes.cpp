#include "raylib.h"
#include "raymath.h"
#include <cmath>
#include <stdio.h>

using namespace std;

#define FISH_WIDTH 10
#define FISH_LENGTH 20
#define FISH_COLOR BLUE
#define FISH_SPEED 70

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 450

#define WALL_BUFFER_DIST 80.0f
#define A_WALL 300.0f

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
    float T_v;
    float T_r;
    float K_phi;
    float tau;
};

struct Fish {
    Kinematics kin;
};

Vector2 desiredWorldAcceleration(const Fish &fish);

Vector2 get_commands(const Vector2 &a_des_world, const Fish &fish,
                     const MotionParams &params);

Kinematics integrate(const Kinematics &kin, Vector2 commands,
                     const MotionParams &params, float dt);

void drawFish(const Fish &fish);

int main() {
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Fishes");
    SetTargetFPS(60);
    float dt = 0.0f;
    float v_cmd = FISH_SPEED;
    float r_cmd = 0;
    Vector2 commands = {v_cmd, r_cmd};
    MotionParams fish_params = MotionParams(10, 100, 4, 0.3, 0.1, 5, 0.3);
    Fish fish = Fish({{50.0, 100.0}, -PI / 4, FISH_SPEED, 0});
    while (!WindowShouldClose()) {
        dt = GetFrameTime();
        Vector2 a_des_world = desiredWorldAcceleration(fish);
        Vector2 commands = get_commands(a_des_world, fish, fish_params);
        fish.kin = integrate(fish.kin, commands, fish_params, dt);
        BeginDrawing();
        ClearBackground(BLACK);
        drawFish(fish);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}

void drawFish(const Fish &fish) {
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

Kinematics integrate(const Kinematics &kin, Vector2 commands,
                     const MotionParams &params, float dt) {
    float v_cmd = commands.x;
    float r_cmd = commands.y;
    float v_dot = (v_cmd - kin.v) / params.T_v;
    float r_dot = (r_cmd - kin.r) / params.T_r;

    float x = kin.pos.x + kin.v * cos(kin.phi) * dt;
    float y = kin.pos.y - kin.v * sin(kin.phi) * dt;
    Vector2 pos = {x, y};
    float phi = kin.phi + kin.r * dt;
    float v = kin.v + v_dot * dt;
    float r = kin.r + r_dot * dt;
    return Kinematics(pos, phi, v, r);
}

inline float wallRamp(float d) {
    return Clamp(1.0f - d / WALL_BUFFER_DIST, 0.0f, 1.0f);
}
Vector2 desiredWorldAcceleration(const Fish &fish) {
    const Vector2 p = fish.kin.pos;
    Vector2 a{0, 0};
    a.x += A_WALL * wallRamp(p.x);
    a.x -= A_WALL * wallRamp(WINDOW_WIDTH - p.x);
    a.y += A_WALL * wallRamp(p.y);
    a.y -= A_WALL * wallRamp(WINDOW_HEIGHT - p.y);
    return a;
}
Vector2 get_commands(const Vector2 &a_des_world, const Fish &fish,
                     const MotionParams &params) {
    Vector2 heading = {cos(fish.kin.phi), -sin(fish.kin.phi)};
    Vector2 v_des = fish.kin.v * heading + a_des_world * params.tau;

    float phi_des = atan2(-v_des.y, v_des.x);
    float e_phi = Wrap(phi_des - fish.kin.phi, -PI, PI);

    float r_cmd = Clamp(params.K_phi * e_phi, -params.r_max, params.r_max);
    float v_cmd = Clamp(hypot(v_des.x, v_des.y), params.v_min, params.v_max);
    return {v_cmd, r_cmd};
};
