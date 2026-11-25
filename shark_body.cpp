#include "shark_body.h"
#include "raymath.h"

namespace {

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

std::vector<Vector2> placeSpine(const std::vector<PathPoint> &path,
                                const BodyParams &params) {
    std::vector<Vector2> spine(params.node_count);
    float nose_arc = path.back().arc;
    int search_from = (int)path.size() - 1;
    for (int i = 0; i < params.node_count; ++i) {
        float s = (float)i / (params.node_count - 1);
        spine[i] = pointAtArc(path, search_from, nose_arc - s * params.length);
    }
    return spine;
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

} // namespace

SharkBody seedSharkBody(const Kinematics &kin, const BodyParams &params) {
    SharkBody body;
    body.path = {{kin.pos - kin.heading * params.length, 0.0f},
                 {kin.pos, params.length}};
    body.spine = placeSpine(body.path, params);
    return body;
}

SharkBody updateSharkBody(const SharkBody &body, const Kinematics &kin,
                          const BodyParams &params, [[maybe_unused]] float dt) {
    SharkBody next = body;
    const PathPoint &nose = body.path.back();
    float step = Vector2Distance(nose.pos, kin.pos);
    if (step > 0.0f)
        next.path.push_back({kin.pos, nose.arc + step});
    trimPath(next.path, params.length);
    next.spine = placeSpine(next.path, params);
    return next;
}
