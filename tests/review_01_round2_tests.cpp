#include "doctest/doctest.h"

#include "shark_body.h"
#include <cmath>
#include <vector>

namespace {

constexpr float L = 3.0f;
constexpr int NODES = 24;
constexpr float TOL = 1e-3f; // m

BodyParams flatParams() {
    return {.length = L,
            .node_count = NODES,
            .wave_amplitude = 0.0f,
            .wave_number = 5.3f,
            .strouhal = 0.3f,
            .length_passes = 1};
}

Kinematics makeKin(Vector2 pos, float phi, float v, float r) {
    return {pos, phi, v, r, {std::cos(phi), -std::sin(phi)}};
}

struct Projection {
    float distance;
    float arc;
};

// Reference polyline of every nose position fed in, with arc accumulated in
// double, independent of the body module's own bookkeeping.
struct Reference {
    std::vector<Vector2> pts;
    std::vector<double> arc;

    void add(Vector2 p) {
        if (pts.empty()) {
            pts.push_back(p);
            arc.push_back(0.0);
            return;
        }
        double d = std::hypot((double)p.x - pts.back().x,
                              (double)p.y - pts.back().y);
        pts.push_back(p);
        arc.push_back(arc.back() + d);
    }

    Projection project(Vector2 q) const {
        Projection best{1e9f, 0.0f};
        for (size_t k = 1; k < pts.size(); ++k) {
            double ax = pts[k - 1].x, ay = pts[k - 1].y;
            double bx = pts[k].x - ax, by = pts[k].y - ay;
            double len2 = bx * bx + by * by;
            double t = len2 > 0 ? ((q.x - ax) * bx + (q.y - ay) * by) / len2 : 0;
            t = std::fmin(1.0, std::fmax(0.0, t));
            double d = std::hypot(q.x - (ax + t * bx), q.y - (ay + t * by));
            if (d < best.distance)
                best = {(float)d,
                        (float)(arc[k - 1] + t * (arc[k] - arc[k - 1]))};
        }
        return best;
    }
};

void checkOnReference(const SharkBody &body, const Reference &ref) {
    REQUIRE(body.spine.size() == NODES);
    double nose_arc = ref.arc.back();
    for (int i = 0; i < NODES; ++i) {
        CAPTURE(i);
        Projection p = ref.project(body.spine[i]);
        CHECK(p.distance < TOL);
        float s = (float)i / (NODES - 1);
        CHECK(std::abs((float)(nose_arc - p.arc) - s * L) < TOL);
    }
}

} // namespace

TEST_CASE("review 01 round 2 R7/R12: varying speed and curvature, uneven dt") {
    const BodyParams params = flatParams();
    const std::vector<float> dts = {1.0f / 30.0f, 0.003f, 1.0f / 90.0f,
                                    0.027f, 0.0001f, 1.0f / 45.0f};
    float phi = 0.9f;
    Kinematics kin = makeKin({15.0f, 8.0f}, phi, 1.0f, 0.0f);

    Reference ref;
    Vector2 tail = {kin.pos.x - kin.heading.x * L,
                    kin.pos.y - kin.heading.y * L};
    ref.add(tail);
    ref.add(kin.pos);
    SharkBody body = seedSharkBody(kin, params);

    float t = 0.0f;
    for (int k = 0; k < 3000; ++k) {
        float dt = dts[k % dts.size()];
        t += dt;
        float v = 1.45f + 1.05f * std::sin(0.7f * t);
        float r = 0.8f * std::sin(0.45f * t);
        Vector2 pos = {kin.pos.x + kin.heading.x * kin.v * dt,
                       kin.pos.y + kin.heading.y * kin.v * dt};
        phi += kin.r * dt;
        kin = makeKin(pos, phi, v, r);
        body = updateSharkBody(body, kin, params, dt);
        ref.add(kin.pos);

        if (k % 250 == 0) {
            CAPTURE(k);
            checkOnReference(body, ref);
        }
    }
    checkOnReference(body, ref);
}

TEST_CASE("review 01 round 2 R5: retained path covers L, nothing older than "
          "needed, while turning") {
    const BodyParams params = flatParams();
    float phi = -0.4f;
    Kinematics kin = makeKin({10.0f, 10.0f}, phi, 0.5f, 0.8f);
    SharkBody body = seedSharkBody(kin, params);
    const float dt = 1.0f / 50.0f;
    for (int k = 0; k < 1500; ++k) {
        Vector2 pos = {kin.pos.x + kin.heading.x * kin.v * dt,
                       kin.pos.y + kin.heading.y * kin.v * dt};
        phi += kin.r * dt;
        kin = makeKin(pos, phi, 0.5f, 0.8f);
        body = updateSharkBody(body, kin, params, dt);

        REQUIRE(body.path.size() >= 2);
        float total = 0.0f;
        for (size_t j = 1; j < body.path.size(); ++j)
            total += std::hypot(body.path[j].pos.x - body.path[j - 1].pos.x,
                                body.path[j].pos.y - body.path[j - 1].pos.y);
        float second = 0.0f;
        for (size_t j = 2; j < body.path.size(); ++j)
            second += std::hypot(body.path[j].pos.x - body.path[j - 1].pos.x,
                                 body.path[j].pos.y - body.path[j - 1].pos.y);
        CHECK(total >= L - 1e-4f);
        CHECK(second < L + 1e-4f);
    }
}

TEST_CASE("review 01 round 2 R12: very small steps keep the spine finite") {
    const BodyParams params = flatParams();
    Kinematics kin = makeKin({5.0f, 5.0f}, 0.0f, 0.5f, 0.0f);
    SharkBody body = seedSharkBody(kin, params);
    for (int k = 0; k < 50; ++k) {
        kin.pos.x += 1e-7f;
        body = updateSharkBody(body, kin, params, 1e-6f);
    }
    for (Vector2 p : body.spine) {
        CHECK(std::isfinite(p.x));
        CHECK(std::isfinite(p.y));
    }
}
