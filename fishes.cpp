#include "raylib.h"
#include "raymath.h"
#include <algorithm>
#include <cmath>
#include <stdio.h>
#include <vector>

using namespace std;

#define N_FISH 1000
#define N_SHARKS 2

#define FISH_WIDTH 10.0f
#define FISH_LENGTH 20.0f
#define FISH_COLOR BLUE

#define FISH_SPEED_MIN 5.0f
#define FISH_SPEED_CRUISE 70.0f
#define FISH_SPEED_MAX 100.0f

#define FISH_SIGHT_RANGE 100.0f
#define FISH_FLEE_RANGE 180.0f
#define FISH_COMFORT_RANGE 40.0f

#define SHARK_WIDTH 20.0f
#define SHARK_LENGTH 40.0f
#define SHARK_COLOR RED

#define SHARK_SPEED_MIN 50.0f
#define SHARK_SPEED_CRUISE 60.0f
#define SHARK_SPEED_MAX 120.0f

#define SHARK_SIGHT_RANGE 200.0f
#define SHARK_COMFORT_RANGE 60.0f
#define SHARK_HUNGER_RATE 0.1f
#define SHARK_EAT_DISTANCE 15.0f
#define SHARK_EAT_DURATION 5.0f

#define WINDOW_WIDTH 1500
#define WINDOW_HEIGHT 800

#define WALL_BUFFER_DIST 100.0f
#define W_WALLS 500.0f
#define T_ALIGN 0.5f
#define T_CRUISE 0.5f
#define W_COHERENCE 2.0f
#define W_SEPERATION 90.0f
#define W_FLEE 300.0f
#define W_CHASE 300.0f

struct Kinematics {
    Vector2 pos{};
    float phi = 0.0f;
    float v = 0.0f;
    float r = 0.0f;
    Vector2 heading = {cos(phi), -sin(phi)};
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
    bool dead;
};

struct Target {
    int index;
    Vector2 relative_pos;
    float distance;
};

struct Hunger {
    float value;
    float time;
};

struct Shark {
    Kinematics kin;
    Target target{-1, {0, 0}, 0};
    Hunger hunger;
};

struct Commands {
    float v_cmd;
    float r_cmd;
};

constexpr float CELL = FISH_SIGHT_RANGE;
constexpr int GX = (int)(WINDOW_WIDTH / CELL) + 1;
constexpr int GY = (int)(WINDOW_HEIGHT / CELL) + 1;

struct Grid {
    vector<int> start =
        vector<int>(GX * GY + 1); // start[c]..start[c+1] = fish in cell c
    vector<int> fill = vector<int>(GX * GY);
    vector<int> items; // fish indices sorted by cell

    static int cx(float x) { return clamp((int)(x / CELL), 0, GX - 1); }
    static int cy(float y) { return clamp((int)(y / CELL), 0, GY - 1); }

    void build(const vector<Fish> &fish) {
        std::fill(start.begin(), start.end(), 0);
        items.resize(fish.size());
        for (const Fish &f : fish)
            start[cy(f.kin.pos.y) * GX + cx(f.kin.pos.x) + 1]++;
        for (int c = 0; c < GX * GY; ++c)
            start[c + 1] += start[c];
        copy(start.begin(), start.end() - 1, fill.begin());
        for (int i = 0; i < (int)fish.size(); ++i) {
            const Vector2 &p = fish[i].kin.pos;
            items[fill[cy(p.y) * GX + cx(p.x)]++] = i;
        }
    }

    template <class F> void forNeighbours(Vector2 p, F &&f) const {
        int x0 = cx(p.x), y0 = cy(p.y);
        for (int y = max(y0 - 1, 0); y <= min(y0 + 1, GY - 1); ++y)
            for (int x = max(x0 - 1, 0); x <= min(x0 + 1, GX - 1); ++x)
                for (int k = start[y * GX + x]; k < start[y * GX + x + 1]; ++k)
                    f(items[k]);
    }
};

Target sharkGetTarget(int i, const vector<Shark> &sharks,
                      const vector<Fish> &fish, const Grid &grid);

Vector2 desiredFishAcceleration(int i, const vector<Shark> &sharks,
                                const vector<Fish> &fish, MotionParams &params,
                                const Grid &grid);

Vector2 desiredSharkAcceleration(int i, const vector<Shark> &sharks,
                                 const vector<Fish> &fish,
                                 MotionParams &params);

Commands getCommands(const Vector2 &a_des_world, const Kinematics &kin,
                     const MotionParams &params);

Commands safetyFilter(Commands cmd, const Kinematics &kin,
                      const MotionParams &params);

Kinematics integrate(const Kinematics &kin, Commands commands,
                     const MotionParams &params, float dt);
Hunger sharkUpdateHunger(const Shark &shark, float dt);

void killFish(vector<Shark> &sharks, vector<Fish> &fish);

void drawFish(const Fish &fish);
void drawShark(const Shark &shark);

int main() {
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Fishes");
    SetTargetFPS(60);

    MotionParams fish_params =
        MotionParams(FISH_SPEED_MIN, FISH_SPEED_CRUISE, FISH_SPEED_MAX, 6, 0.2,
                     0.05, 5, 0.3);

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
        f.dead = false;
    }

    for (Shark &s : sharks_old) {
        s.kin = {{(float)GetRandomValue(100, WINDOW_WIDTH - 100),
                  (float)GetRandomValue(100, WINDOW_HEIGHT - 100)},
                 GetRandomValue(0, 359) * DEG2RAD,
                 SHARK_SPEED_CRUISE,
                 0.0f};
        s.hunger.value = 0.0f;
        s.hunger.time = 0.0f;
    }

    Grid grid;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        grid.build(fish_old);

#pragma omp parallel for

        for (int i = 0; i < (int)fish_old.size(); ++i) {
            fish_new[i] = fish_old[i];
            Vector2 a = desiredFishAcceleration(i, sharks_old, fish_old,
                                                fish_params, grid);
            Commands cmd = getCommands(a, fish_old[i].kin, fish_params);
            cmd = safetyFilter(cmd, fish_old[i].kin, fish_params);
            fish_new[i].kin = integrate(fish_old[i].kin, cmd, fish_params, dt);
            fish_new[i].dead = fish_old[i].dead;
        }
#pragma omp parallel for

        for (int i = 0; i < (int)sharks_old.size(); ++i) {
            sharks_new[i] = sharks_old[i];
            Vector2 a =
                desiredSharkAcceleration(i, sharks_old, fish_old, shark_params);
            Commands cmd = getCommands(a, sharks_old[i].kin, shark_params);
            cmd = safetyFilter(cmd, sharks_old[i].kin, shark_params);
            sharks_new[i].kin =
                integrate(sharks_old[i].kin, cmd, shark_params, dt);
            sharks_new[i].hunger = sharkUpdateHunger(sharks_old[i], dt);
            sharks_new[i].target =
                sharkGetTarget(i, sharks_old, fish_old, grid);
        }

        killFish(sharks_new, fish_new);

        BeginDrawing();
        ClearBackground(BLACK);
        for (const Fish &f : fish_new)
            if (!f.dead) {
                drawFish(f);
            }
        for (const Shark &s : sharks_new)
            drawShark(s);
        EndDrawing();

        swap(fish_old, fish_new);
        swap(sharks_old, sharks_new);
    }
    CloseWindow();
    return 0;
}
Hunger sharkUpdateHunger(const Shark &shark, float dt) {
    Hunger hunger;
    hunger.time = shark.hunger.time + dt;
    hunger.value = shark.hunger.value;
    if (hunger.time > SHARK_EAT_DURATION) {
        hunger.value = Clamp(hunger.value + SHARK_HUNGER_RATE * dt, 0, 1);
    }
    return hunger;
}
void killFish(vector<Shark> &sharks, vector<Fish> &fish) {
    for (Shark &s : sharks) {
        if (s.target.index < 0) {
            continue;
        }
        Fish &target = fish[s.target.index];
        if ((s.target.distance < SHARK_EAT_DISTANCE) &&
            (abs(Vector2Angle(s.kin.heading, target.kin.heading)) <
             45 * DEG2RAD) &&
            (s.hunger.time > SHARK_EAT_DURATION)) {
            target.dead = true;
            s.hunger.value = 0.0;
            s.hunger.time = 0.0;
        }
    }
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

    Vector2 pos = kin.pos + kin.v * kin.heading * dt;

    float phi = kin.phi + kin.r * dt;
    float v = kin.v + v_dot * dt;
    float r = kin.r + r_dot * dt;
    return Kinematics(pos, phi, v, r);
}

inline float wallRamp(float d) {
    return Clamp(1.0f - d / WALL_BUFFER_DIST, 0.0f, 1.0f);
}

Target sharkGetTarget(int i, const vector<Shark> &sharks,
                      const vector<Fish> &fish, const Grid &grid) {
    const Kinematics &self = sharks[i].kin;
    int closest = -1;
    float closest_distance = SHARK_SIGHT_RANGE;
    Vector2 closest_relative_pos = {0.0f, 0.0f};
    grid.forNeighbours(self.pos, [&](int f) {
        if (fish[f].dead) {
            return;
        }
        const Kinematics &candidate = fish[f].kin;
        Vector2 candidate_relative_pos = candidate.pos - self.pos;
        float distance = Vector2Length(candidate_relative_pos);
        if (distance < closest_distance && distance < SHARK_SIGHT_RANGE) {
            closest = f;
            closest_distance = distance;
            closest_relative_pos = candidate_relative_pos;
        }
    });
    return {closest, closest_relative_pos, closest_distance};
}

Vector2 desiredSharkAcceleration(int i, const vector<Shark> &sharks,
                                 const vector<Fish> &fish,
                                 MotionParams &params) {

    const Shark &self = sharks[i];
    Vector2 a_sep{}, a_chase{};

    for (int j = 0; j < sharks.size(); j++) {
        if (j == i) {
            continue;
        }
        const Kinematics &other = sharks[j].kin;
        Vector2 relative_pos = other.pos - self.kin.pos;
        float distance = Vector2Length(relative_pos);
        if (distance < (SHARK_COMFORT_RANGE) && distance > 1e-6f) {

            a_sep -= relative_pos * (1.0f - distance / SHARK_COMFORT_RANGE) /
                     distance;
        }
    }

    Vector2 a_walls{0, 0};
    a_walls.x += wallRamp(self.kin.pos.x);
    a_walls.x -= wallRamp(WINDOW_WIDTH - self.kin.pos.x);
    a_walls.y += wallRamp(self.kin.pos.y);
    a_walls.y -= wallRamp(WINDOW_HEIGHT - self.kin.pos.y);

    Vector2 a_cruise = self.kin.heading * (params.v_cruise - self.kin.v);

    if (self.target.index >= 0) {
        a_chase = self.target.relative_pos *
                  (1.0f - self.target.distance / SHARK_SIGHT_RANGE) /
                  self.target.distance;
    }
    Vector2 a_des = W_WALLS * a_walls + a_cruise / T_CRUISE +
                    W_SEPERATION * a_sep +
                    W_CHASE * self.hunger.value * a_chase;
    return a_des;
}

Vector2 desiredFishAcceleration(int i, const vector<Shark> &sharks,
                                const vector<Fish> &fish, MotionParams &params,
                                const Grid &grid) {

    const Kinematics &self = fish[i].kin;

    Vector2 a_sep{}, a_coh{}, a_align{}, a_flee{}, sum_pos{}, sum_vel{};

    int adj_fish = 0;
    grid.forNeighbours(self.pos, [&](int j) {
        if ((j == i) || fish[j].dead) {
            return;
        }
        const Kinematics &other = fish[j].kin;
        Vector2 relative_pos = other.pos - self.pos;
        float distance = Vector2Length(relative_pos);
        if (distance > (FISH_SIGHT_RANGE)) {
            return;
        }
        if (distance < (FISH_COMFORT_RANGE) && distance > 1e-6f) {

            a_sep -= relative_pos * (1.0f - distance / FISH_COMFORT_RANGE) /
                     distance;
        }
        adj_fish++;
        sum_pos += relative_pos;
        sum_vel += Vector2{cos(other.phi), -sin(other.phi)} * other.v;
    });

    if (adj_fish > 0) {
        a_coh = sum_pos / adj_fish;
        a_align = (sum_vel / adj_fish) - self.heading * self.v;
    }

    Vector2 a_walls{0, 0};
    a_walls.x += wallRamp(self.pos.x);
    a_walls.x -= wallRamp(WINDOW_WIDTH - self.pos.x);
    a_walls.y += wallRamp(self.pos.y);
    a_walls.y -= wallRamp(WINDOW_HEIGHT - self.pos.y);

    Vector2 a_cruise = self.heading * (params.v_cruise - self.v);

    for (const Shark &s : sharks) {
        Vector2 relative_pos = s.kin.pos - self.pos;
        float distance = Vector2Length(relative_pos);
        if (distance > (FISH_FLEE_RANGE)) {
            continue;
        }
        a_flee -= relative_pos * (1.0f - distance / FISH_FLEE_RANGE) / distance;
    }

    Vector2 a_des = W_WALLS * a_walls + a_align / T_ALIGN +
                    a_cruise / T_CRUISE + W_COHERENCE * a_coh +
                    W_SEPERATION * a_sep + W_FLEE * Vector2Normalize(a_flee);
    return a_des;
}
Commands getCommands(const Vector2 &a_des_world, const Kinematics &kin,
                     const MotionParams &params) {
    Vector2 v_des = kin.v * kin.heading + a_des_world * params.tau;

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

Vector2 nearWallNormal(Vector2 p) {
    Vector2 n{0, 0};
    if (p.x < WALL_BUFFER_DIST)
        n.x += 1;
    if (p.x > WINDOW_WIDTH - WALL_BUFFER_DIST)
        n.x -= 1;
    if (p.y < WALL_BUFFER_DIST)
        n.y += 1;
    if (p.y > WINDOW_HEIGHT - WALL_BUFFER_DIST)
        n.y -= 1;
    return n;
}

Commands safetyFilter(Commands cmd, const Kinematics &kin,
                      const MotionParams &params) {
    Vector2 fwd = kin.heading;
    Vector2 left = {fwd.y, -fwd.x};
    float R = kin.v / params.r_max;
    float margin = kin.v * params.T_r + 20.0f;

    float clearPos = circleClearance(kin.pos + left * R, R);
    float clearNeg = circleClearance(kin.pos - left * R, R);

    Vector2 n = nearWallNormal(kin.pos);
    bool heading_out = Vector2DotProduct(fwd, n) > 0.0f;
    if (heading_out || max(clearPos, clearNeg) >= margin)
        return cmd;
    float turn;
    if (fabsf(clearPos - clearNeg) < 1.0f)
        turn = (kin.r >= 0.0f) ? 1.0f : -1.0f;
    else
        turn = (clearPos > clearNeg) ? 1.0f : -1.0f;

    return {min(cmd.v_cmd, params.v_cruise), turn * params.r_max};
}
