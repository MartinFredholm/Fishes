#include "shark_body.h"
#include "raymath.h"
#include <array>
#include <cmath>

namespace {

struct Knot {
    float s;
    float value;
};

// Fritsch–Carlson monotone cubic Hermite: smooth (C1) and, unlike a natural
// cubic spline, never overshoots the knot values.
template <size_t N>
float interpolateSmooth(const std::array<Knot, N> &knots, float s) {
    if (s <= knots.front().s)
        return knots.front().value;
    if (s >= knots.back().s)
        return knots.back().value;
    size_t j = 0;
    while (knots[j + 1].s < s)
        ++j;
    auto secant = [&](size_t k) {
        return (knots[k + 1].value - knots[k].value) /
               (knots[k + 1].s - knots[k].s);
    };
    auto slope = [&](size_t k) {
        if (k == 0)
            return secant(0);
        if (k == N - 1)
            return secant(N - 2);
        float left = secant(k - 1), right = secant(k);
        if (left * right <= 0.0f)
            return 0.0f;
        return 2.0f / (1.0f / left + 1.0f / right);
    };
    float h = knots[j + 1].s - knots[j].s;
    float t = (s - knots[j].s) / h;
    float t2 = t * t, t3 = t2 * t;
    return (2 * t3 - 3 * t2 + 1) * knots[j].value +
           (t3 - 2 * t2 + t) * h * slope(j) +
           (-2 * t3 + 3 * t2) * knots[j + 1].value +
           (t3 - t2) * h * slope(j + 1);
}

// Source: https://julialang.org/blog/2021/08/sharks/
constexpr std::array<Knot, 6> WAVE_ENVELOPE = {
    {{0.0f, 0.2f}, {0.2f, 0.21f}, {0.4f, 0.23f},
     {0.6f, 0.4f}, {0.8f, 0.88f}, {1.0f, 1.0f}}};

float pathLengthNeeded(const BodyParams &params) {
    return (1.0f - params.pivot) * params.length;
}

std::vector<Vector2> straightPathBehind(const Kinematics &kin,
                                        const BodyParams &params) {
    return {kin.pos - kin.heading * pathLengthNeeded(params), kin.pos};
}

void trimPath(std::vector<Vector2> &path, float needed) {
    float covered = 0.0f;
    for (size_t k = path.size() - 1; k > 0; --k) {
        covered += Vector2Distance(path[k], path[k - 1]);
        if (covered >= needed) {
            path.erase(path.begin(), path.begin() + (k - 1));
            return;
        }
    }
}

std::vector<Vector2> extendPath(const std::vector<Vector2> &path,
                                const Kinematics &kin,
                                const BodyParams &params) {
    // A default-constructed body has no path yet; it starts as at spawn.
    if (path.empty())
        return straightPathBehind(kin, params);
    std::vector<Vector2> extended = path;
    if (kin.pos.x != path.back().x || kin.pos.y != path.back().y)
        extended.push_back(kin.pos);
    trimPath(extended, pathLengthNeeded(params));
    return extended;
}

// Path layout: oldest point first, kin.pos last.
Vector2 pointBehindOnPath(const std::vector<Vector2> &path, float behind) {
    for (size_t k = path.size() - 1; k > 0; --k) {
        if (behind <= 0.0f)
            return path[k];
        float segment = Vector2Distance(path[k], path[k - 1]);
        if (behind <= segment)
            return Vector2Lerp(path[k], path[k - 1], behind / segment);
        behind -= segment;
    }
    // Reached only by float rounding in the path's length.
    return path[0] + Vector2Normalize(path[0] - path[1]) * behind;
}

// kappa in 1/m, with the sign of r. Written with sin(x)/x and (1 - cos x)/x
// so it stays exact and finite as kappa goes to 0.
Vector2 pointOnHeadArc(const Kinematics &kin, float kappa, float ahead) {
    // A denormal v overflows r/v; the arc's radius then goes to 0.
    if (std::isinf(kappa))
        return kin.pos;
    float bend = kappa * ahead;
    float along = ahead, across = 0.0f;
    if (bend != 0.0f) {
        along = ahead * std::sin(bend) / bend;
        across = ahead * 2.0f * std::sin(bend / 2.0f) * std::sin(bend / 2.0f) /
                 bend;
    }
    Vector2 toward_positive_turn = {kin.heading.y, -kin.heading.x};
    return kin.pos + kin.heading * along + toward_positive_turn * across;
}

std::vector<Vector2> placeSpine(const std::vector<Vector2> &path,
                                const Kinematics &kin,
                                const BodyParams &params) {
    float kappa = kin.v > 0.0f ? kin.r / kin.v : 0.0f;
    std::vector<Vector2> spine(params.node_count);
    for (int i = 0; i < params.node_count; ++i) {
        float s = (float)i / (params.node_count - 1);
        float behind_pivot = (s - params.pivot) * params.length;
        spine[i] = behind_pivot >= 0.0f
                       ? pointBehindOnPath(path, behind_pivot)
                       : pointOnHeadArc(kin, kappa, -behind_pivot);
    }
    return spine;
}

Vector2 spineNormal(const std::vector<Vector2> &spine, int i) {
    int front = i > 0 ? i - 1 : i;
    int back = i + 1 < (int)spine.size() ? i + 1 : i;
    Vector2 tangent = Vector2Normalize(spine[front] - spine[back]);
    return {-tangent.y, tangent.x};
}

std::vector<Vector2> applyWave(const std::vector<Vector2> &spine, float phase,
                               const BodyParams &params) {
    std::vector<Vector2> waved(spine.size());
    for (int i = 0; i < (int)spine.size(); ++i) {
        float s = (float)i / (params.node_count - 1);
        float h = params.wave_amplitude * params.length *
                  interpolateSmooth(WAVE_ENVELOPE, s) *
                  std::sin(params.wave_number * s - phase);
        waved[i] = spine[i] + spineNormal(spine, i) * h;
    }
    return waved;
}

void restoreSegmentLengths(std::vector<Vector2> &spine,
                           const BodyParams &params) {
    float rest = params.length / (params.node_count - 1);
    for (int pass = 0; pass < params.length_passes; ++pass)
        for (size_t i = 1; i < spine.size(); ++i)
            spine[i] = spine[i - 1] +
                       Vector2Normalize(spine[i] - spine[i - 1]) * rest;
}

float advancePhase(float phase, float speed, const BodyParams &params,
                   float dt) {
    float omega = 2.0f * PI * params.strouhal * speed /
                  (2.0f * params.wave_amplitude * params.length);
    return std::remainder(phase + omega * dt, 2.0f * PI);
}

// With zero amplitude omega is undefined, and a length pass would pull the
// spine's chords off the arc.
bool waveOff(const BodyParams &params) { return params.wave_amplitude == 0.0f; }

std::vector<Vector2> buildSpine(const std::vector<Vector2> &path,
                                const Kinematics &kin, float phase,
                                const BodyParams &params) {
    std::vector<Vector2> spine = placeSpine(path, kin, params);
    if (waveOff(params))
        return spine;
    spine = applyWave(spine, phase, params);
    restoreSegmentLengths(spine, params);
    return spine;
}

} // namespace

SharkBody seedSharkBody(const Kinematics &kin, const BodyParams &params) {
    SharkBody body;
    body.pivot_path = straightPathBehind(kin, params);
    body.spine = buildSpine(body.pivot_path, kin, body.phase, params);
    return body;
}

SharkBody updateSharkBody(const SharkBody &body, const Kinematics &kin,
                          const BodyParams &params, float dt) {
    SharkBody next;
    next.phase = waveOff(params) ? body.phase
                                 : advancePhase(body.phase, kin.v, params, dt);
    next.pivot_path = extendPath(body.pivot_path, kin, params);
    next.spine = buildSpine(next.pivot_path, kin, next.phase, params);
    return next;
}
