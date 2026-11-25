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

// kappa in 1/m, with the sign of r. Written with sin(x)/x and (1 - cos x)/x
// so it stays exact and finite as kappa goes to 0.
Vector2 pointOnTurnBend(Vector2 nose, Vector2 heading, float kappa,
                        float behind) {
    float bend = kappa * behind;
    float along = behind, across = 0.0f;
    if (bend != 0.0f) {
        along = behind * std::sin(bend) / bend;
        across = behind * 2.0f * std::sin(bend / 2.0f) * std::sin(bend / 2.0f) /
                 bend;
    }
    Vector2 toward_positive_turn = {heading.y, -heading.x};
    return nose - heading * along + toward_positive_turn * across;
}

std::vector<Vector2> bendSpine(const Kinematics &kin,
                               const BodyParams &params) {
    float kappa = kin.v > 0.0f ? kin.r / kin.v : 0.0f;
    // A denormal v overflows r/v; the arc's radius then goes to 0.
    if (std::isinf(kappa))
        return std::vector<Vector2>(params.node_count, kin.pos);
    std::vector<Vector2> spine(params.node_count);
    for (int i = 0; i < params.node_count; ++i) {
        float s = (float)i / (params.node_count - 1);
        spine[i] = pointOnTurnBend(kin.pos, kin.heading, kappa, s * params.length);
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

std::vector<Vector2> buildSpine(const Kinematics &kin, float phase,
                                const BodyParams &params) {
    std::vector<Vector2> spine = bendSpine(kin, params);
    if (waveOff(params))
        return spine;
    spine = applyWave(spine, phase, params);
    restoreSegmentLengths(spine, params);
    return spine;
}

} // namespace

SharkBody seedSharkBody(const Kinematics &kin, const BodyParams &params) {
    SharkBody body;
    body.spine = buildSpine(kin, body.phase, params);
    return body;
}

SharkBody updateSharkBody(const SharkBody &body, const Kinematics &kin,
                          const BodyParams &params, float dt) {
    SharkBody next;
    next.phase = waveOff(params) ? body.phase
                                 : advancePhase(body.phase, kin.v, params, dt);
    next.spine = buildSpine(kin, next.phase, params);
    return next;
}
