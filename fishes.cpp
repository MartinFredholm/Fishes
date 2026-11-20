#include "raylib.h"
#include "raymath.h"
#include <cmath>
#include <stdio.h>
#include <vector>

using namespace std;

#define N 1000

#define FISH_WIDTH 10.0f
#define FISH_LENGTH 20.0f
#define FISH_COLOR BLUE

#define FISH_SPEED_MIN 10.0f
#define FISH_SPEED_CRUISE 60.0f
#define FISH_SPEED_MAX 100.0f

#define FISH_SIGHT_RANGE 50.0f
#define FISH_COMFORT_RANGE 25.0f

#define WINDOW_WIDTH 1500
#define WINDOW_HEIGHT 800

#define WALL_BUFFER_DIST 100.0f
#define W_WALLS 300.0f
#define T_ALIGN 0.5f
#define T_CRUISE 0.5f
#define W_COHERENCE 1.0f
#define W_SEPERATION 100.0f

struct Kinematics {
    Vector2 pos;
    float phi;
    float v;
    float r;
};

struct MotionParams {
    float v_min;
    float v_cruise;
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

Vector2 desiredWorldAcceleration(int i, const vector<Fish> &fish,
                                 MotionParams &params);

Vector2 getCommands(const Vector2 &a_des_world, const Fish &fish,
                    const MotionParams &params);

Kinematics integrate(const Kinematics &kin, Vector2 commands,
                     const MotionParams &params, float dt);

void drawFish(const Fish &fish);

int main() {
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Fishes");
    SetTargetFPS(60);
    MotionParams fish_params = MotionParams(
        FISH_SPEED_MIN, FISH_SPEED_CRUISE, FISH_SPEED_MAX, 4, 0.3, 0.1, 5, 0.3);
    vector<Fish> fish_old(N), fish_new(N);
    for (Fish &f : fish_old) {
        f.kin = {{(float)GetRandomValue(100, WINDOW_WIDTH - 100),
                  (float)GetRandomValue(100, WINDOW_HEIGHT - 100)},
                 GetRandomValue(0, 359) * DEG2RAD,
                 FISH_SPEED_CRUISE,
                 0.0f};
    }
    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        for (int i = 0; i < (int)fish_old.size(); ++i) {
            Vector2 a = desiredWorldAcceleration(i, fish_old, fish_params);
            Vector2 cmd = getCommands(a, fish_old[i], fish_params);
            fish_new[i].kin = integrate(fish_old[i].kin, cmd, fish_params, dt);
        }
        std::swap(fish_old, fish_new);
        BeginDrawing();
        ClearBackground(BLACK);
        for (const Fish &f : fish_old)
            drawFish(f);
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

Vector2 desiredWorldAcceleration(int i, const vector<Fish> &fish,
                                 MotionParams &params) {

    const Kinematics &self = fish[i].kin;
    Vector2 heading = {cos(self.phi), -sin(self.phi)};

    Vector2 a_sep{}, a_coh{}, a_align{}, sum_pos{}, sum_vel{};

    int adj_fish = 0;
    for (int j = 0; j < fish.size(); j++) {
        if (j == i) {
            continue;
        }
        const Kinematics &other = fish[j].kin;
        Vector2 relative_pos = other.pos - self.pos;
        float distance = Vector2Length(relative_pos);
        if (distance > (FISH_SIGHT_RANGE)) {
            continue;
        }
        if (distance < (FISH_COMFORT_RANGE) && distance > 1e-6f) {

            a_sep -= relative_pos * (1.0f - distance / FISH_COMFORT_RANGE) /
                     distance;
        }
        adj_fish++;
        sum_pos += relative_pos;
        sum_vel += Vector2{cos(other.phi), -sin(other.phi)} * other.v;
    }

    if (adj_fish > 0) {
        a_coh = sum_pos / adj_fish;
        a_align = (sum_vel / adj_fish) - heading * self.v;
    }

    Vector2 a_walls{0, 0};
    a_walls.x += wallRamp(self.pos.x);
    a_walls.x -= wallRamp(WINDOW_WIDTH - self.pos.x);
    a_walls.y += wallRamp(self.pos.y);
    a_walls.y -= wallRamp(WINDOW_HEIGHT - self.pos.y);

    Vector2 a_cruise = heading * (params.v_cruise - self.v);

    Vector2 a_des = W_WALLS * a_walls + a_align / T_ALIGN +
                    a_cruise / T_CRUISE + W_COHERENCE * a_coh +
                    W_SEPERATION * a_sep;
    return a_des;
}
Vector2 getCommands(const Vector2 &a_des_world, const Fish &fish,
                    const MotionParams &params) {
    Vector2 heading = {cos(fish.kin.phi), -sin(fish.kin.phi)};
    Vector2 v_des = fish.kin.v * heading + a_des_world * params.tau;

    float phi_des = atan2(-v_des.y, v_des.x);
    float e_phi = Wrap(phi_des - fish.kin.phi, -PI, PI);

    float r_cmd = Clamp(params.K_phi * e_phi, -params.r_max, params.r_max);
    float v_cmd = Clamp(hypot(v_des.x, v_des.y), params.v_min, params.v_max);
    return {v_cmd, r_cmd};
};
