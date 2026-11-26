#include "kinematics.h"
#include "motion.h"
#include "raylib.h"
#include "raymath.h"
#include "shark_avoidance.h"
#include "shark_body.h"
#include <algorithm>
#include <cmath>
#include <vector>

constexpr int N_FISH = 4000;
constexpr int N_SHARKS = 2;

constexpr float FISH_WIDTH = 0.1f;
constexpr float FISH_LENGTH = 0.2f;
constexpr Color FISH_COLOR = BLUE;

constexpr MotionParams FISH_MOTION = {.v_min = 0.15f,
                                      .v_cruise = 0.6f,
                                      .v_max = 1.5f,
                                      .r_max = 6.0f,
                                      .T_v = 0.2f,
                                      .T_r = 0.05f,
                                      .K_phi = 5.0f,
                                      .tau = 0.3f,
                                      .wall_clearance = 0.0f};

constexpr float FISH_SIGHT_RANGE = 1.4f;
constexpr float FISH_FLEE_RANGE = 2.5f;
constexpr float FISH_COMFORT_RANGE = 0.5f;

constexpr float SHARK_LENGTH = 3.0f;
constexpr Color SHARK_COLOR = {140, 128, 106, 255};

constexpr float SHARK_HALF_SPAN_AT_PECTORAL_TIPS = 0.2f * SHARK_LENGTH;

constexpr MotionParams SHARK_MOTION = {.v_min = 0.5f,
                                       .v_cruise = 1.0f,
                                       .v_max = 2.5f,
                                       .r_max = 0.8f,
                                       .T_v = 1.0f,
                                       .T_r = 0.4f,
                                       .K_phi = 4.0f,
                                       .tau = 0.3f,
                                       .wall_clearance =
                                           SHARK_HALF_SPAN_AT_PECTORAL_TIPS};

constexpr BodyParams SHARK_BODY = {.length = SHARK_LENGTH,
                                   .node_count = 24,
                                   .wave_amplitude = 0.1f,
                                   .wave_number = 5.3f,
                                   .strouhal = 0.3f,
                                   .length_passes = 1};

constexpr float SHARK_SIGHT_RANGE = 6.0f;
constexpr float SHARK_COMFORT_RANGE = 6.0f;
constexpr float SHARK_HUNGER_RATE = 0.05f;
constexpr float SHARK_EAT_DISTANCE = 0.3f;
constexpr float SHARK_EAT_DURATION = 5.0f;

constexpr float SPAWN_MARGIN = 1.0f;
constexpr float MAX_DT = 1.0f / 30.0f;

constexpr float WALL_BUFFER_DIST = 1.0f;
constexpr float SAFETY_MARGIN = 0.2f;
// W_COHERENCE is in 1/s^2; the other W_ weights are in m/s^2.
constexpr float W_WALLS = 5.0f;
constexpr float T_ALIGN = 0.5f;
constexpr float T_CRUISE = 0.5f;
constexpr float W_COHERENCE = 2.0f;
constexpr float W_FISH_SEPARATION = 0.9f;
constexpr float W_SHARK_SEPARATION = 4.0f;
constexpr float W_FLEE = 3.0f;
constexpr float W_CHASE = 3.0f;

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
    SharkBody body;
};

constexpr float GRID_CELL_SIZE = 1.0f;
constexpr int GX = (int)(WORLD_WIDTH / GRID_CELL_SIZE) + 1;
constexpr int GY = (int)(WORLD_HEIGHT / GRID_CELL_SIZE) + 1;

struct Grid {
    std::vector<int> cell_start = std::vector<int>(GX * GY + 1);
    std::vector<int> fill = std::vector<int>(GX * GY);
    std::vector<int> fish_by_cell;

    static int cx(float x) {
        return std::clamp((int)(x / GRID_CELL_SIZE), 0, GX - 1);
    }
    static int cy(float y) {
        return std::clamp((int)(y / GRID_CELL_SIZE), 0, GY - 1);
    }

    void build(const std::vector<Fish> &fish) {
        std::fill(cell_start.begin(), cell_start.end(), 0);
        fish_by_cell.resize(fish.size());
        for (const Fish &f : fish)
            cell_start[cy(f.kin.pos.y) * GX + cx(f.kin.pos.x) + 1]++;
        for (int c = 0; c < GX * GY; ++c)
            cell_start[c + 1] += cell_start[c];
        std::copy(cell_start.begin(), cell_start.end() - 1, fill.begin());
        for (int i = 0; i < (int)fish.size(); ++i) {
            const Vector2 &p = fish[i].kin.pos;
            fish_by_cell[fill[cy(p.y) * GX + cx(p.x)]++] = i;
        }
    }

    template <class F>
    void forFishInCellsAround(Vector2 p, float radius, F &&f) const {
        int cell_reach = (int)std::ceil(radius / GRID_CELL_SIZE);
        int x0 = cx(p.x), y0 = cy(p.y);
        for (int y = std::max(y0 - cell_reach, 0);
             y <= std::min(y0 + cell_reach, GY - 1); ++y)
            for (int x = std::max(x0 - cell_reach, 0);
                 x <= std::min(x0 + cell_reach, GX - 1); ++x)
                for (int k = cell_start[y * GX + x];
                     k < cell_start[y * GX + x + 1]; ++k)
                    f(fish_by_cell[k]);
    }
};

Target sharkGetTarget(int i, const std::vector<Shark> &sharks,
                      const std::vector<Fish> &fish, const Grid &grid);

Vector2 desiredFishAcceleration(int i, const std::vector<Shark> &sharks,
                                const std::vector<Fish> &fish,
                                const MotionParams &params, const Grid &grid);

Vector2 desiredSharkAcceleration(int i, const std::vector<Shark> &sharks,
                                 const MotionParams &params);

Commands getCommands(const Vector2 &a_des_world, const Kinematics &kin,
                     const MotionParams &params);

std::vector<OtherShark> otherSharks(int i, const std::vector<Shark> &sharks);

Commands safetyFilter(Commands cmd, const Kinematics &kin,
                      const MotionParams &params);

Hunger sharkUpdateHunger(const Shark &shark, float dt);

void killFish(std::vector<Shark> &sharks, std::vector<Fish> &fish);

void drawBody(const Kinematics &kin, float length, float width, Color color);

void drawShark(const SharkBody &body, Color color);

float randomUniform(float lo, float hi) {
    constexpr int RESOLUTION = 1 << 20;
    return lo + (hi - lo) * (float)GetRandomValue(0, RESOLUTION) / RESOLUTION;
}

Kinematics randomStart(float v) {
    return {{randomUniform(SPAWN_MARGIN, WORLD_WIDTH - SPAWN_MARGIN),
             randomUniform(SPAWN_MARGIN, WORLD_HEIGHT - SPAWN_MARGIN)},
            GetRandomValue(0, 359) * DEG2RAD,
            v,
            0.0f};
}

int main() {
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Fishes");
    SetTargetFPS(60);

    std::vector<Fish> fish_old(N_FISH), fish_new(N_FISH);
    std::vector<Shark> sharks_old(N_SHARKS), sharks_new(N_SHARKS);

    for (Fish &f : fish_old) {
        f.kin = randomStart(FISH_MOTION.v_cruise);
        f.dead = false;
    }

    for (Shark &s : sharks_old) {
        s.kin = randomStart(SHARK_MOTION.v_cruise);
        s.hunger.value = 0.0f;
        s.hunger.time = 0.0f;
        s.body = seedSharkBody(s.kin, SHARK_BODY);
    }

    Grid grid;

    while (!WindowShouldClose()) {
        float dt = std::min(GetFrameTime(), MAX_DT);
        grid.build(fish_old);

#pragma omp parallel for
        for (int i = 0; i < (int)fish_old.size(); ++i) {
            fish_new[i] = fish_old[i];
            if (fish_old[i].dead) {
                continue;
            }
            Vector2 a = desiredFishAcceleration(i, sharks_old, fish_old,
                                                FISH_MOTION, grid);
            Commands cmd = getCommands(a, fish_old[i].kin, FISH_MOTION);
            cmd = safetyFilter(cmd, fish_old[i].kin, FISH_MOTION);
            fish_new[i].kin = integrate(fish_old[i].kin, cmd, FISH_MOTION, dt);
        }

        for (int i = 0; i < (int)sharks_old.size(); ++i) {
            sharks_new[i] = sharks_old[i];
            Vector2 a = desiredSharkAcceleration(i, sharks_old, SHARK_MOTION);
            Commands cmd = getCommands(a, sharks_old[i].kin, SHARK_MOTION);
            cmd = avoidSharks(sharks_old[i].kin, cmd,
                              otherSharks(i, sharks_old), SHARK_MOTION);
            cmd = safetyFilter(cmd, sharks_old[i].kin, SHARK_MOTION);
            sharks_new[i].kin =
                integrate(sharks_old[i].kin, cmd, SHARK_MOTION, dt);
            sharks_new[i].body = updateSharkBody(
                sharks_old[i].body, sharks_new[i].kin, SHARK_BODY, dt);
            sharks_new[i].hunger = sharkUpdateHunger(sharks_old[i], dt);
            sharks_new[i].target =
                sharkGetTarget(i, sharks_old, fish_old, grid);
        }

        killFish(sharks_new, fish_new);

        BeginDrawing();
        ClearBackground(BLACK);
        for (const Fish &f : fish_new)
            if (!f.dead) {
                drawBody(f.kin, FISH_LENGTH, FISH_WIDTH, FISH_COLOR);
            }
        for (const Shark &s : sharks_new)
            drawShark(s.body, SHARK_COLOR);
        EndDrawing();

        std::swap(fish_old, fish_new);
        std::swap(sharks_old, sharks_new);
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
void killFish(std::vector<Shark> &sharks, std::vector<Fish> &fish) {
    for (Shark &s : sharks) {
        if (s.target.index < 0) {
            continue;
        }
        Fish &target = fish[s.target.index];
        if ((s.target.distance < SHARK_EAT_DISTANCE) &&
            (std::abs(Vector2Angle(s.kin.heading, target.kin.heading)) <
             45 * DEG2RAD) &&
            (s.hunger.time > SHARK_EAT_DURATION)) {
            target.dead = true;
            s.hunger.value = 0.0;
            s.hunger.time = 0.0;
        }
    }
}

Vector2 toPixels(Vector2 p) { return p / METERS_PER_PIXEL; }

void drawBody(const Kinematics &kin, float length, float width, Color color) {
    Vector2 fwd = kin.heading;
    Vector2 left = {fwd.y, -fwd.x};
    Vector2 tail = kin.pos - fwd * (length / 2);

    Vector2 v_back_1 = tail + left * (width / 2);
    Vector2 v_back_2 = tail - left * (width / 2);
    Vector2 v_front = kin.pos + fwd * (length / 2);

    DrawTriangle(toPixels(v_back_1), toPixels(v_back_2), toPixels(v_front),
                 color);
}

// Right before left gives counter-clockwise triangles on screen, which raylib
// needs to not cull them.
void drawShark(const SharkBody &body, Color color) {
    BodyShape shape = buildBodyShape(body.spine, SHARK_BODY.length);
    std::vector<Vector2> strip;
    strip.reserve(2 * shape.left.size());
    for (size_t i = 0; i < shape.left.size(); ++i) {
        strip.push_back(toPixels(shape.right[i]));
        strip.push_back(toPixels(shape.left[i]));
    }
    DrawTriangleStrip(strip.data(), (int)strip.size(), color);
    for (const std::vector<Vector2> &part : shape.rigid_parts) {
        std::vector<Vector2> fan;
        fan.reserve(part.size());
        for (Vector2 p : part)
            fan.push_back(toPixels(p));
        DrawTriangleFan(fan.data(), (int)fan.size(), color);
    }
}

inline float wallRamp(float d) {
    return Clamp(1.0f - d / WALL_BUFFER_DIST, 0.0f, 1.0f);
}

Vector2 wallRepulsion(Vector2 pos, float clearance) {
    return {wallRamp(pos.x - clearance) -
                wallRamp(WORLD_WIDTH - pos.x - clearance),
            wallRamp(pos.y - clearance) -
                wallRamp(WORLD_HEIGHT - pos.y - clearance)};
}

Target sharkGetTarget(int i, const std::vector<Shark> &sharks,
                      const std::vector<Fish> &fish, const Grid &grid) {
    const Kinematics &self = sharks[i].kin;
    int closest = -1;
    float closest_distance = SHARK_SIGHT_RANGE;
    Vector2 closest_relative_pos = {0.0f, 0.0f};
    grid.forFishInCellsAround(self.pos, SHARK_SIGHT_RANGE, [&](int f) {
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

std::vector<OtherShark> otherSharks(int i, const std::vector<Shark> &sharks) {
    std::vector<OtherShark> others;
    others.reserve(sharks.size() - 1);
    for (int j = 0; j < (int)sharks.size(); ++j)
        if (j != i)
            others.push_back({sharks[j].kin, sharks[j].body.body_line});
    return others;
}

Vector2 desiredSharkAcceleration(int i, const std::vector<Shark> &sharks,
                                 const MotionParams &params) {

    const Shark &self = sharks[i];
    Vector2 a_sep{}, a_chase{};

    for (int j = 0; j < (int)sharks.size(); j++) {
        if (j == i) {
            continue;
        }
        Vector2 relative_pos =
            nearestPointOnBodyLine(self.kin.pos, sharks[j].body.body_line) -
            self.kin.pos;
        float distance = Vector2Length(relative_pos);
        if (distance < (SHARK_COMFORT_RANGE) && distance > 1e-6f) {

            a_sep -= relative_pos * (1.0f - distance / SHARK_COMFORT_RANGE) /
                     distance;
        }
    }

    Vector2 a_walls = wallRepulsion(self.kin.pos, params.wall_clearance);

    Vector2 a_cruise = self.kin.heading * (params.v_cruise - self.kin.v);

    if (self.target.index >= 0 && self.target.distance > 1e-6f) {
        a_chase = self.target.relative_pos *
                  (1.0f - self.target.distance / SHARK_SIGHT_RANGE) /
                  self.target.distance;
    }
    Vector2 a_des = W_WALLS * a_walls + a_cruise / T_CRUISE +
                    W_SHARK_SEPARATION * a_sep +
                    W_CHASE * self.hunger.value * a_chase;
    return a_des;
}

Vector2 desiredFishAcceleration(int i, const std::vector<Shark> &sharks,
                                const std::vector<Fish> &fish,
                                const MotionParams &params, const Grid &grid) {

    const Kinematics &self = fish[i].kin;

    Vector2 a_sep{}, a_coh{}, a_align{}, a_flee{}, sum_pos{}, sum_vel{};

    int adj_fish = 0;
    grid.forFishInCellsAround(self.pos, FISH_SIGHT_RANGE, [&](int j) {
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
        sum_vel += other.heading * other.v;
    });

    if (adj_fish > 0) {
        a_coh = sum_pos / adj_fish;
        a_align = (sum_vel / adj_fish) - self.heading * self.v;
    }

    Vector2 a_walls = wallRepulsion(self.pos, params.wall_clearance);

    Vector2 a_cruise = self.heading * (params.v_cruise - self.v);

    for (const Shark &s : sharks) {
        Vector2 relative_pos =
            nearestPointOnBodyLine(self.pos, s.body.body_line) - self.pos;
        float distance = Vector2Length(relative_pos);
        if (distance > (FISH_FLEE_RANGE) || distance < 1e-6f) {
            continue;
        }
        a_flee -= relative_pos * (1.0f - distance / FISH_FLEE_RANGE) / distance;
    }

    Vector2 a_des = W_WALLS * a_walls + a_align / T_ALIGN +
                    a_cruise / T_CRUISE + W_COHERENCE * a_coh +
                    W_FISH_SEPARATION * a_sep + W_FLEE * Vector2Normalize(a_flee);
    return a_des;
}
Commands getCommands(const Vector2 &a_des_world, const Kinematics &kin,
                     const MotionParams &params) {
    Vector2 v_des = kin.v * kin.heading + a_des_world * params.tau;

    float phi_des = std::atan2(-v_des.y, v_des.x);
    float e_phi = Wrap(phi_des - kin.phi, -PI, PI);

    float r_cmd = Clamp(params.K_phi * e_phi, -params.r_max, params.r_max);
    float v_cmd =
        Clamp(std::hypot(v_des.x, v_des.y), params.v_min, params.v_max);
    return {v_cmd, r_cmd};
};

float circleClearance(Vector2 c, float R, float wall_clearance) {
    return std::min({c.x - R, WORLD_WIDTH - c.x - R, c.y - R,
                     WORLD_HEIGHT - c.y - R}) -
           wall_clearance;
}

Vector2 nearWallNormal(Vector2 p, float clearance) {
    float buffer = WALL_BUFFER_DIST + clearance;
    Vector2 n{0, 0};
    if (p.x < buffer)
        n.x += 1;
    if (p.x > WORLD_WIDTH - buffer)
        n.x -= 1;
    if (p.y < buffer)
        n.y += 1;
    if (p.y > WORLD_HEIGHT - buffer)
        n.y -= 1;
    return n;
}

Commands safetyFilter(Commands cmd, const Kinematics &kin,
                      const MotionParams &params) {
    Vector2 fwd = kin.heading;
    Vector2 left = {fwd.y, -fwd.x};
    float R = kin.v / params.r_max;
    float margin = kin.v * params.T_r + SAFETY_MARGIN;

    float clearPos =
        circleClearance(kin.pos + left * R, R, params.wall_clearance);
    float clearNeg =
        circleClearance(kin.pos - left * R, R, params.wall_clearance);

    Vector2 n = nearWallNormal(kin.pos, params.wall_clearance);
    bool heading_out = Vector2DotProduct(fwd, n) > 0.0f;
    if (heading_out || std::max(clearPos, clearNeg) >= margin)
        return cmd;
    float turn;
    if (std::abs(clearPos - clearNeg) < CLEARANCE_TIE)
        turn = (kin.r >= 0.0f) ? 1.0f : -1.0f;
    else
        turn = (clearPos > clearNeg) ? 1.0f : -1.0f;

    return {std::min(cmd.v_cmd, params.v_cruise), turn * params.r_max};
}
