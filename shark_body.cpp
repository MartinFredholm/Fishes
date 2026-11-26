#include "shark_body.h"
#include "raymath.h"
#include <algorithm>
#include <array>
#include <cmath>

namespace {

struct Knot {
    float s;
    float value;
};

// Fritsch–Carlson: C1 and, unlike a natural cubic spline, never overshoots.
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

constexpr std::array<Knot, 8> WIDTH_OVER_LENGTH = {
    {{0.00f, 0.05f}, {0.10f, 0.09f}, {0.25f, 0.12f}, {0.40f, 0.11f},
     {0.60f, 0.07f}, {0.75f, 0.035f}, {0.90f, 0.02f}, {1.00f, 0.005f}}};

struct PartVertex {
    float back_over_length;
    float left_over_length;
};

using PartOutline = std::vector<PartVertex>;

// The leading-edge curve between the spec's span, tip and chord points is a
// guess. The leading edge lies ahead of the attachment so that it covers the
// outline's blunt front; the fan therefore starts and ends at the leading-edge
// midline to close around the attachment.
const PartOutline CEPHALOFOIL_OUTLINE = {
    {0.00f, 0.0f},
    {-0.01f, 0.0f},    {0.01f, 0.08f},    {0.04f, 0.125f},  {0.06f, 0.135f},
    {0.07f, 0.13f},    {0.05f, 0.0f},     {0.07f, -0.13f},  {0.06f, -0.135f},
    {0.04f, -0.125f},  {0.01f, -0.08f},   {-0.01f, 0.0f}};

struct FinShape {
    float root_s;
    float root_end_s;
    float tip_s;
    float span_from_edge;
};

constexpr FinShape PECTORAL_FIN = {0.26f, 0.32f, 0.38f, 0.14f};
constexpr FinShape PELVIC_FIN = {0.54f, 0.57f, 0.60f, 0.04f};

Vector2 pointAtArc(const std::vector<PathPoint> &path, int &search_from,
                   float arc) {
    while (search_from > 0 && path[search_from - 1].arc > arc)
        --search_from;
    if (search_from == 0)
        return path.front().pos;
    const PathPoint &a = path[search_from - 1];
    const PathPoint &b = path[search_from];
    float t = (arc - a.arc) / (b.arc - a.arc);
    return Vector2Lerp(a.pos, b.pos, t);
}

std::vector<Vector2> placeAtUniformS(const std::vector<PathPoint> &path,
                                     int count, float length) {
    std::vector<Vector2> points(count);
    float nose_arc = path.back().arc;
    int search_from = (int)path.size() - 1;
    for (int i = 0; i < count; ++i) {
        float s = (float)i / (count - 1);
        points[i] = pointAtArc(path, search_from, nose_arc - s * length);
    }
    return points;
}

// Rebasing arc to the oldest point keeps float arc values small however far
// the shark swims.
void trimPath(std::vector<PathPoint> &path, float length) {
    float tail_arc = path.back().arc - length;
    auto first_kept = path.begin();
    while (first_kept + 1 != path.end() && (first_kept + 1)->arc <= tail_arc)
        ++first_kept;
    path.erase(path.begin(), first_kept);
    float base = path.front().arc;
    for (PathPoint &p : path)
        p.arc -= base;
}

// Second-order differences at the ends: a first-order end chord tilts the wide
// nose outline by more than 1 mm in tight turns.
Vector2 spineLeftNormal(const std::vector<Vector2> &spine, int i) {
    int last = (int)spine.size() - 1;
    Vector2 noseward;
    if (i == 0)
        noseward = spine[0] * 3.0f - spine[1] * 4.0f + spine[2];
    else if (i == last)
        noseward =
            spine[last - 1] * 4.0f - spine[last] * 3.0f - spine[last - 2];
    else
        noseward = spine[i - 1] - spine[i + 1];
    Vector2 tangent = Vector2Normalize(noseward);
    return {tangent.y, -tangent.x};
}

float halfWidth(float s) { return interpolateSmooth(WIDTH_OVER_LENGTH, s) / 2; }

// The vertex on the spine closes the fin over the body so no gap opens in
// bends.
PartOutline leftFinOutline(const FinShape &fin) {
    float root_half = halfWidth(fin.root_s);
    return {{0.0f, 0.0f},
            {0.0f, root_half},
            {fin.tip_s - fin.root_s, root_half + fin.span_from_edge},
            {fin.root_end_s - fin.root_s, halfWidth(fin.root_end_s)}};
}

// Reversing all but the first vertex keeps the winding and the fan centre.
PartOutline mirrored(const PartOutline &outline) {
    PartOutline mirror = {
        {outline[0].back_over_length, -outline[0].left_over_length}};
    for (size_t i = outline.size() - 1; i > 0; --i)
        mirror.push_back(
            {outline[i].back_over_length, -outline[i].left_over_length});
    return mirror;
}

Vector2 spineTailward(const std::vector<Vector2> &spine, int i) {
    Vector2 left = spineLeftNormal(spine, i);
    return {left.y, -left.x};
}

struct SpineFrame {
    Vector2 origin;
    Vector2 back;
};

// Hermite, not linear: between 24 nodes the chord sags 3 mm off a 0.625 m
// turn.
SpineFrame spineFrameAt(const std::vector<Vector2> &spine, float s) {
    int last = (int)spine.size() - 1;
    float x = s * last;
    int j = std::min((int)x, last - 1);
    float t = x - j;
    Vector2 p0 = spine[j], p1 = spine[j + 1];
    float chord = Vector2Distance(p0, p1);
    Vector2 m0 = spineTailward(spine, j) * chord;
    Vector2 m1 = spineTailward(spine, j + 1) * chord;
    float t2 = t * t, t3 = t2 * t;
    Vector2 origin = p0 * (2 * t3 - 3 * t2 + 1) + m0 * (t3 - 2 * t2 + t) +
                     p1 * (-2 * t3 + 3 * t2) + m1 * (t3 - t2);
    Vector2 velocity = p0 * (6 * t2 - 6 * t) + m0 * (3 * t2 - 4 * t + 1) +
                       p1 * (-6 * t2 + 6 * t) + m1 * (3 * t2 - 2 * t);
    return {origin, Vector2Normalize(velocity)};
}

std::vector<Vector2> placePart(const std::vector<Vector2> &spine, float s,
                               const PartOutline &outline, float length) {
    SpineFrame frame = spineFrameAt(spine, s);
    Vector2 origin = frame.origin;
    Vector2 back = frame.back;
    Vector2 left = {-back.y, back.x};
    std::vector<Vector2> part;
    part.reserve(outline.size());
    for (PartVertex v : outline)
        part.push_back(origin + back * (v.back_over_length * length) +
                       left * (v.left_over_length * length));
    return part;
}

std::vector<Vector2> applyWave(const std::vector<Vector2> &spine, float phase,
                               const BodyParams &params) {
    std::vector<Vector2> waved(spine.size());
    for (int i = 0; i < (int)spine.size(); ++i) {
        float s = (float)i / (params.node_count - 1);
        float h = params.wave_amplitude * params.length *
                  interpolateSmooth(WAVE_ENVELOPE, s) *
                  std::sin(params.wave_number * s - phase);
        waved[i] = spine[i] + spineLeftNormal(spine, i) * h;
    }
    return waved;
}

void restoreSegmentLengths(std::vector<Vector2> &spine,
                           const BodyParams &params) {
    float rest = params.length / (params.node_count - 1);
    for (int pass = 0; pass < params.length_passes; ++pass) {
        const std::vector<Vector2> waved = spine;
        for (size_t i = 1; i < spine.size(); ++i)
            spine[i] = spine[i - 1] +
                       Vector2Normalize(waved[i] - waved[i - 1]) * rest;
    }
}

float advancePhase(float phase, float speed, const BodyParams &params,
                   float dt) {
    float omega = 2.0f * PI * params.strouhal * speed /
                  (2.0f * params.wave_amplitude * params.length);
    return std::remainder(phase + omega * dt, 2.0f * PI);
}

} // namespace

SharkBody seedSharkBody(const Kinematics &kin, const BodyParams &params) {
    SharkBody body;
    body.path = {{kin.pos - kin.heading * params.length, 0.0f},
                 {kin.pos, params.length}};
    body.spine = placeAtUniformS(body.path, params.node_count, params.length);
    body.body_line =
        placeAtUniformS(body.path, BODY_LINE_POINTS, params.length);
    return body;
}

SharkBody updateSharkBody(const SharkBody &body, const Kinematics &kin,
                          const BodyParams &params, float dt) {
    SharkBody next = body;
    const PathPoint &nose = body.path.back();
    float step = Vector2Distance(nose.pos, kin.pos);
    if (step > 0.0f)
        next.path.push_back({kin.pos, nose.arc + step});
    trimPath(next.path, params.length);
    next.spine = placeAtUniformS(next.path, params.node_count, params.length);
    next.body_line =
        placeAtUniformS(next.path, BODY_LINE_POINTS, params.length);
    // With zero amplitude omega is undefined, and a length pass would pull the
    // spine's chords off a curved path.
    if (params.wave_amplitude == 0.0f)
        return next;
    next.phase = advancePhase(body.phase, kin.v, params, dt);
    next.spine = applyWave(next.spine, next.phase, params);
    restoreSegmentLengths(next.spine, params);
    return next;
}

Vector2 nearestPointOnBodyLine(Vector2 p,
                               const std::vector<Vector2> &body_line) {
    Vector2 nearest = body_line.front();
    float nearest_dist_sq = Vector2DistanceSqr(p, nearest);
    for (size_t i = 1; i < body_line.size(); ++i) {
        Vector2 a = body_line[i - 1];
        Vector2 segment = body_line[i] - a;
        float length_sq = Vector2LengthSqr(segment);
        float t = length_sq > 0.0f
                      ? Clamp(Vector2DotProduct(p - a, segment) / length_sq,
                              0.0f, 1.0f)
                      : 0.0f;
        Vector2 foot = a + segment * t;
        float dist_sq = Vector2DistanceSqr(p, foot);
        if (dist_sq < nearest_dist_sq) {
            nearest = foot;
            nearest_dist_sq = dist_sq;
        }
    }
    return nearest;
}

BodyShape buildBodyShape(const std::vector<Vector2> &spine, float length) {
    BodyShape shape;
    int n = (int)spine.size();
    shape.left.resize(n);
    shape.right.resize(n);
    for (int i = 0; i < n; ++i) {
        float s = (float)i / (n - 1);
        float half_width = interpolateSmooth(WIDTH_OVER_LENGTH, s) * length / 2;
        Vector2 left = spineLeftNormal(spine, i);
        shape.left[i] = spine[i] + left * half_width;
        shape.right[i] = spine[i] - left * half_width;
    }
    PartOutline pectoral = leftFinOutline(PECTORAL_FIN);
    PartOutline pelvic = leftFinOutline(PELVIC_FIN);
    shape.rigid_parts.resize(BodyShape::RIGID_PART_COUNT);
    shape.rigid_parts[BodyShape::CEPHALOFOIL] =
        placePart(spine, 0.0f, CEPHALOFOIL_OUTLINE, length);
    shape.rigid_parts[BodyShape::PECTORAL_LEFT] =
        placePart(spine, PECTORAL_FIN.root_s, pectoral, length);
    shape.rigid_parts[BodyShape::PECTORAL_RIGHT] =
        placePart(spine, PECTORAL_FIN.root_s, mirrored(pectoral), length);
    shape.rigid_parts[BodyShape::PELVIC_LEFT] =
        placePart(spine, PELVIC_FIN.root_s, pelvic, length);
    shape.rigid_parts[BodyShape::PELVIC_RIGHT] =
        placePart(spine, PELVIC_FIN.root_s, mirrored(pelvic), length);
    return shape;
}
