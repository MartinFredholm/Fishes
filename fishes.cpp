#include "raylib.h"
#include "raymath.h"
#include <cmath>
#include <stdio.h>
#include <vector>

using namespace std;

#define N_FISH 1000
#define N_SHARKS 2

#define FISH_WIDTH 10.0f
#define FISH_LENGTH 20.0f
#define FISH_COLOR BLUE

#define FISH_SPEED_MIN 10.0f
#define FISH_SPEED_CRUISE 60.0f
#define FISH_SPEED_MAX 100.0f

#define FISH_SIGHT_RANGE 100.0f
#define FISH_COMFORT_RANGE 25.0f

#define SHARK_WIDTH 20.0f
#define SHARK_LENGTH 40.0f
#define SHARK_COLOR RED

#define SHARK_SPEED_MIN 20.0f
#define SHARK_SPEED_CRUISE 70.0f
#define SHARK_SPEED_MAX 120.0f

#define SHARK_SIGHT_RANGE 90.0f
#define SHARK_COMFORT_RANGE 40.0f

#define WINDOW_WIDTH 1500
#define WINDOW_HEIGHT 800

#define WALL_BUFFER_DIST 100.0f
#define W_WALLS 300.0f
#define T_ALIGN 1.0f
#define T_CRUISE 0.5f
#define W_COHERENCE 2.0f
#define W_SEPERATION 100.0f
#define W_FLEE 300.0f
#define W_CHASE 300.0f

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

struct Shark {
    Kinematics kin;
};

struct Commands {
    float v_cmd;
    float r_cmd;
};

Vector2 desiredFishAcceleration(int i, const vector<Shark> &sharks,
                                const vector<Fish> &fish, MotionParams &params);

Vector2 desiredSharkAcceleration(int i, const vector<Shark> &sharks,
                                 const vector<Fish> &fish,
                                 MotionParams &params);

Commands getCommands(const Vector2 &a_des_world, const Kinematics &kin,
                     const MotionParams &params);

Commands safetyFilter(Commands cmd, const Kinematics &k,
                      const MotionParams &params);
Kinematics integrate(const Kinematics &kin, Commands commands,
                     const MotionParams &params, float dt);

void drawFish(const Fish &fish);
void drawShark(const Shark &shark);

int main() {
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Fishes");
    SetTargetFPS(60);

    MotionParams fish_params = MotionParams(
        FISH_SPEED_MIN, FISH_SPEED_CRUISE, FISH_SPEED_MAX, 4, 0.3, 0.1, 5, 0.3);

    MotionParams shark_params =
        MotionParams(SHARK_SPEED_MIN, SHARK_SPEED_CRUISE, SHARK_SPEED_MAX, 3,
                     0.6, 0.1, 4, 0.3);

    vector<Fish> fish_old(N_FISH), fish_new(N_FISH);
    vector<Shark> sharks_old(N_SHARKS), sharks_new(N_SHARKS);

    for (Fish &f : fish_old) {
        f.kin = {{(float)GetRandomValue(100, WINDOW_WIDTH - 100),
                  (float)GetRandomValue(100, WINDOW_HEIGHT - 100)},
                 GetRandomValue(0, 359) * DEG2RAD,
                 FISH_SPEED_CRUISE,
                 0.0f};
    }
    for (Shark &s : sharks_old) {
        s.kin = {{(float)GetRandomValue(100, WINDOW_WIDTH - 100),
                  (float)GetRandomValue(100, WINDOW_HEIGHT - 100)},
                 GetRandomValue(0, 359) * DEG2RAD,
                 SHARK_SPEED_CRUISE,
                 0.0f};
    }
    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        for (int i = 0; i < (int)fish_old.size(); ++i) {
            Vector2 a =
                desiredFishAcceleration(i, sharks_old, fish_old, fish_params);
            Commands cmd = getCommands(a, fish_old[i].kin, fish_params);
            cmd = safetyFilter(cmd, fish_old[i].kin, fish_params);
            fish_new[i].kin = integrate(fish_old[i].kin, cmd, fish_params, dt);
        }

        for (int i = 0; i < (int)sharks_old.size(); ++i) {
            Vector2 a =
                desiredSharkAcceleration(i, sharks_old, fish_old, shark_params);
            Commands cmd = getCommands(a, sharks_old[i].kin, shark_params);
            cmd = safetyFilter(cmd, sharks_old[i].kin, shark_params);
            sharks_new[i].kin =
                integrate(sharks_old[i].kin, cmd, shark_params, dt);
        }

        swap(fish_old, fish_new);
        swap(sharks_old, sharks_new);

        BeginDrawing();
        ClearBackground(BLACK);
        for (const Fish &f : fish_old)
            drawFish(f);
        for (const Shark &s : sharks_old)
            drawShark(s);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}

void drawShark(const Shark &shark) {
    Vector2 v_back_1 = {
        shark.kin.pos.x - cos(shark.kin.phi) * SHARK_LENGTH / 2 -
            sin(shark.kin.phi) * SHARK_WIDTH / 2,
        shark.kin.pos.y + sin(shark.kin.phi) * SHARK_LENGTH / 2 -
            cos(shark.kin.phi) * SHARK_WIDTH / 2};

    Vector2 v_back_2 = {
        shark.kin.pos.x - cos(shark.kin.phi) * SHARK_LENGTH / 2 +
            sin(shark.kin.phi) * SHARK_WIDTH / 2,
        shark.kin.pos.y + sin(shark.kin.phi) * SHARK_LENGTH / 2 +
            cos(shark.kin.phi) * SHARK_WIDTH / 2};

    Vector2 v_front = {shark.kin.pos.x + cos(shark.kin.phi) * SHARK_LENGTH / 2,
                       shark.kin.pos.y - sin(shark.kin.phi) * SHARK_LENGTH / 2};

    DrawTriangle(v_back_1, v_back_2, v_front, SHARK_COLOR);
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

Kinematics integrate(const Kinematics &kin, Commands commands,
                     const MotionParams &params, float dt) {
    float v_cmd = commands.v_cmd;
    float r_cmd = commands.r_cmd;
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

Vector2 desiredSharkAcceleration(int i, const vector<Shark> &sharks,
                                 const vector<Fish> &fish,
                                 MotionParams &params) {

    const Kinematics &self = sharks[i].kin;
    Vector2 heading = {cos(self.phi), -sin(self.phi)};

    Vector2 a_sep{}, a_chase{};

    for (int j = 0; j < sharks.size(); j++) {
        if (j == i) {
            continue;
        }
        const Kinematics &other = sharks[j].kin;
        Vector2 relative_pos = other.pos - self.pos;
        float distance = Vector2Length(relative_pos);
        if (distance < (FISH_COMFORT_RANGE) && distance > 1e-6f) {

            a_sep -= relative_pos * (1.0f - distance / SHARK_COMFORT_RANGE) /
                     distance;
        }
    }

    Vector2 a_walls{0, 0};
    a_walls.x += wallRamp(self.pos.x);
    a_walls.x -= wallRamp(WINDOW_WIDTH - self.pos.x);
    a_walls.y += wallRamp(self.pos.y);
    a_walls.y -= wallRamp(WINDOW_HEIGHT - self.pos.y);

    Vector2 a_cruise = heading * (params.v_cruise - self.v);

    int closest = -1;
    float closest_dist = INFINITY;
    for (int f = 0; f < fish.size(); f++) {
        const Kinematics &target = fish[f].kin;
        Vector2 relative_pos = target.pos - self.pos;
        float distance = Vector2Length(relative_pos);
        if (distance < closest_dist && distance < SHARK_SIGHT_RANGE) {
            closest = f;
            closest_dist = distance;
            a_chase =
                relative_pos * (1.0f - distance / SHARK_SIGHT_RANGE) / distance;
        }
    }

    Vector2 a_des = W_WALLS * a_walls + a_cruise / T_CRUISE +
                    W_SEPERATION * a_sep + W_CHASE * a_chase;
    return a_des;
}

Vector2 desiredFishAcceleration(int i, const vector<Shark> &sharks,
                                const vector<Fish> &fish,
                                MotionParams &params) {

    const Kinematics &self = fish[i].kin;
    Vector2 heading = {cos(self.phi), -sin(self.phi)};

    Vector2 a_sep{}, a_coh{}, a_align{}, a_flee{}, sum_pos{}, sum_vel{};

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

    for (const Shark &s : sharks) {
        Vector2 relative_pos = s.kin.pos - self.pos;
        float distance = Vector2Length(relative_pos);
        if (distance > (FISH_SIGHT_RANGE)) {
            continue;
        }
        a_flee -=
            relative_pos * (1.0f - distance / FISH_SIGHT_RANGE) / distance;
    }

    Vector2 a_des = W_WALLS * a_walls + a_align / T_ALIGN +
                    a_cruise / T_CRUISE + W_COHERENCE * a_coh +
                    W_SEPERATION * a_sep + W_FLEE * Vector2Normalize(a_flee);
    return a_des;
}
Commands getCommands(const Vector2 &a_des_world, const Kinematics &kin,
                     const MotionParams &params) {
    Vector2 heading = {cos(kin.phi), -sin(kin.phi)};
    Vector2 v_des = kin.v * heading + a_des_world * params.tau;

    float phi_des = atan2(-v_des.y, v_des.x);
    float e_phi = Wrap(phi_des - kin.phi, -PI, PI);

    float r_cmd = Clamp(params.K_phi * e_phi, -params.r_max, params.r_max);
    float v_cmd = Clamp(hypot(v_des.x, v_des.y), params.v_min, params.v_max);
    return {v_cmd, r_cmd};
};

float circleClearance(Vector2 c, float R) {
    return std::min(
        {c.x - R, WINDOW_WIDTH - c.x - R, c.y - R, WINDOW_HEIGHT - c.y - R});
}

Commands safetyFilter(Commands cmd, const Kinematics &k,
                      const MotionParams &params) {
    Vector2 fwd = {std::cos(k.phi), -std::sin(k.phi)};
    Vector2 norm_fwd = {fwd.y, -fwd.x};
    float R = k.v / params.r_max;
    float margin = k.v * params.T_r + 5.0f;

    float clearPos = circleClearance(k.pos + norm_fwd * R, R);
    float clearNeg = circleClearance(k.pos - norm_fwd * R, R);

    if (std::max(clearPos, clearNeg) < margin) {
        float turn =
            (clearPos >= clearNeg) ? 1.0f : -1.0f; // pick the roomier circle
        cmd = {params.v_min, turn * params.r_max};
    }
    return cmd;
}
