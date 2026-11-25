#include "doctest/doctest.h"

#include "shark_body.h"
#include <cmath>
#include <limits>
#include <vector>

namespace {

constexpr float L = 3.0f;
constexpr int NODES = 24;
constexpr float TOL = 1e-3f; // m
constexpr float V_MIN = 0.5f, V_MAX = 2.5f, R_MAX = 0.8f;

BodyParams bodyWithAmplitude(float amplitude) {
    return {.length = L,
            .node_count = NODES,
            .wave_amplitude = amplitude,
            .wave_number = 5.3f,
            .strouhal = 0.3f,
            .length_passes = 1};
}

Kinematics makeKin(Vector2 pos, float phi, float v, float r) {
    return {pos, phi, v, r, {std::cos(phi), -std::sin(phi)}};
}

float nodeS(int i) { return (float)i / (NODES - 1); }

// The nose's steady-turn track, swum backwards in time from kin with constant
// r and v. Midpoint heading per step; y is down, so heading = (cos phi, -sin phi).
struct Track {
    std::vector<double> x, y, arc;

    explicit Track(const Kinematics &kin) {
        const double ds = 1e-4;
        double px = kin.pos.x, py = kin.pos.y, phi = kin.phi;
        double dphi = kin.v > 0 ? kin.r / kin.v * ds : 0.0;
        x.push_back(px);
        y.push_back(py);
        arc.push_back(0.0);
        while (arc.back() < 1.2 * L) {
            double mid = phi - dphi / 2;
            px -= std::cos(mid) * ds;
            py += std::sin(mid) * ds;
            phi -= dphi;
            x.push_back(px);
            y.push_back(py);
            arc.push_back(arc.back() + ds);
        }
    }

    struct Projection {
        double distance;
        double arc_behind;
    };

    Projection project(Vector2 q) const {
        Projection best{1e9, 0.0};
        for (size_t k = 1; k < x.size(); ++k) {
            double bx = x[k] - x[k - 1], by = y[k] - y[k - 1];
            double t = ((q.x - x[k - 1]) * bx + (q.y - y[k - 1]) * by) /
                       (bx * bx + by * by);
            t = std::fmin(1.0, std::fmax(0.0, t));
            double d = std::hypot(q.x - (x[k - 1] + t * bx),
                                  q.y - (y[k - 1] + t * by));
            if (d < best.distance)
                best = {d, arc[k - 1] + t * (arc[k] - arc[k - 1])};
        }
        return best;
    }
};

void checkOnTurnBendArc(const SharkBody &body, const Kinematics &kin) {
    REQUIRE(body.spine.size() == NODES);
    Track track(kin);
    for (int i = 0; i < NODES; ++i) {
        CAPTURE(i);
        Track::Projection p = track.project(body.spine[i]);
        CHECK(p.distance < TOL);
        CHECK(std::abs(p.arc_behind - nodeS(i) * L) < TOL);
    }
}

} // namespace

TEST_CASE("R7: zero amplitude, every node on the turn bend's arc at s_i L") {
    const BodyParams params = bodyWithAmplitude(0.0f);
    struct Case {
        float v, r;
    };
    const Case cases[] = {{1.0f, 0.0f},    {V_MIN, R_MAX}, {V_MIN, -R_MAX},
                          {V_MAX, R_MAX},  {V_MAX, -R_MAX}, {1.0f, 0.3f},
                          {1.7f, -0.45f}};
    for (const Case &c : cases) {
        for (float phi : {0.0f, 2.2f, -1.3f}) {
            CAPTURE(c.v);
            CAPTURE(c.r);
            CAPTURE(phi);
            Kinematics kin = makeKin({12.0f, 7.0f}, phi, c.v, c.r);
            SharkBody body = seedSharkBody(makeKin({2.0f, 3.0f}, 0.4f, 1.0f, 0.0f),
                                           params);
            body = updateSharkBody(body, kin, params, 1.0f / 60.0f);
            checkOnTurnBendArc(body, kin);
        }
    }
}

TEST_CASE("R6: first update after spawn at cruise moves no node more than "
          "0.05 m, and the seed already carries the wave at phase 0") {
    const BodyParams params = bodyWithAmplitude(0.1f);
    const float dt = 1.0f / 60.0f;
    for (float phi : {0.0f, 2.3f, -0.9f}) {
        CAPTURE(phi);
        Kinematics kin = makeKin({7.0f, 4.0f}, phi, 1.0f, 0.0f);
        SharkBody seed = seedSharkBody(kin, params);
        REQUIRE(seed.spine.size() == NODES);
        CHECK(seed.phase == 0.0f);

        SharkBody at_phase_zero;
        at_phase_zero.phase = 0.0f;
        SharkBody ordinary = updateSharkBody(at_phase_zero, kin, params, 0.0f);
        for (int i = 0; i < NODES; ++i) {
            CAPTURE(i);
            CHECK(std::hypot(seed.spine[i].x - ordinary.spine[i].x,
                             seed.spine[i].y - ordinary.spine[i].y) < 1e-6f);
        }

        Kinematics moved = makeKin({kin.pos.x + kin.heading.x * kin.v * dt,
                                    kin.pos.y + kin.heading.y * kin.v * dt},
                                   phi, 1.0f, 0.0f);
        SharkBody first = updateSharkBody(seed, moved, params, dt);
        for (int i = 0; i < NODES; ++i) {
            CAPTURE(i);
            CHECK(std::hypot(first.spine[i].x - seed.spine[i].x,
                             first.spine[i].y - seed.spine[i].y) <= 0.05f);
        }
    }
}

TEST_CASE("R19: different swimming histories, same phase, same kin and dt "
          "give the same spine") {
    const BodyParams params = bodyWithAmplitude(0.1f);
    const float dt = 1.0f / 60.0f;

    Kinematics straight = makeKin({4.0f, 4.0f}, 0.2f, 1.0f, 0.0f);
    SharkBody a = seedSharkBody(straight, params);
    for (int step = 0; step < 300; ++step) {
        straight.pos = {straight.pos.x + straight.heading.x * straight.v * dt,
                        straight.pos.y + straight.heading.y * straight.v * dt};
        a = updateSharkBody(a, straight, params, dt);
    }

    Kinematics turning = makeKin({20.0f, 10.0f}, -1.0f, 1.8f, R_MAX);
    SharkBody b = seedSharkBody(turning, params);
    float phi = turning.phi;
    for (int step = 0; step < 300; ++step) {
        float r = step < 150 ? R_MAX : -0.5f;
        Vector2 pos = {turning.pos.x + turning.heading.x * turning.v * dt,
                       turning.pos.y + turning.heading.y * turning.v * dt};
        phi += r * dt;
        turning = makeKin(pos, phi, 1.8f, r);
        b = updateSharkBody(b, turning, params, dt);
    }

    a.phase = b.phase = 1.3f;
    const Kinematics now = makeKin({9.0f, 6.0f}, 0.8f, 0.7f, -0.6f);
    SharkBody next_a = updateSharkBody(a, now, params, dt);
    SharkBody next_b = updateSharkBody(b, now, params, dt);
    REQUIRE(next_a.spine.size() == NODES);
    REQUIRE(next_b.spine.size() == NODES);
    for (int i = 0; i < NODES; ++i) {
        CAPTURE(i);
        CHECK(std::hypot(next_a.spine[i].x - next_b.spine[i].x,
                         next_a.spine[i].y - next_b.spine[i].y) < 1e-4f);
    }
}

TEST_CASE("R20: v = 0 gives a straight spine along kin.heading at s_i L") {
    const BodyParams params = bodyWithAmplitude(0.0f);
    for (float r : {0.0f, R_MAX, -R_MAX}) {
        CAPTURE(r);
        const float phi = 1.9f;
        Kinematics kin = makeKin({6.0f, 9.0f}, phi, 0.0f, r);
        SharkBody body = updateSharkBody(seedSharkBody(kin, params), kin,
                                         params, 1.0f / 60.0f);
        REQUIRE(body.spine.size() == NODES);
        Vector2 back = {-std::cos(phi), std::sin(phi)};
        for (int i = 0; i < NODES; ++i) {
            CAPTURE(i);
            Vector2 expected = {kin.pos.x + back.x * nodeS(i) * L,
                                kin.pos.y + back.y * nodeS(i) * L};
            CHECK(std::hypot(body.spine[i].x - expected.x,
                             body.spine[i].y - expected.y) < TOL);
        }
    }
}

TEST_CASE("R20: v from 0 up to v_min at r = +-r_max keeps every coordinate "
          "finite") {
    const float speeds[] = {0.0f,  std::numeric_limits<float>::denorm_min(),
                            1e-38f, 1e-30f, 1e-12f, 1e-6f, 1e-3f, 0.05f,
                            0.25f,  V_MIN};
    for (float amplitude : {0.0f, 0.1f}) {
        const BodyParams params = bodyWithAmplitude(amplitude);
        for (float v : speeds) {
            for (float r : {R_MAX, -R_MAX}) {
                CAPTURE(amplitude);
                CAPTURE(v);
                CAPTURE(r);
                Kinematics kin = makeKin({6.0f, 9.0f}, 0.6f, v, r);
                SharkBody body = seedSharkBody(kin, params);
                for (int step = 0; step < 3; ++step)
                    body = updateSharkBody(body, kin, params, 1.0f / 60.0f);
                REQUIRE(body.spine.size() == NODES);
                for (Vector2 p : body.spine) {
                    CHECK(std::isfinite(p.x));
                    CHECK(std::isfinite(p.y));
                }
            }
        }
    }
}

TEST_CASE("R12: uneven dt with v and r changing every step keeps R9 and R10") {
    const BodyParams params = bodyWithAmplitude(0.1f);
    const float uneven[] = {1.0f / 30.0f, 0.0f,         0.004f, 1.0f / 144.0f,
                            0.021f,       0.0005f,      0.0f,   1.0f / 33.0f};
    const double pi = 3.14159265358979323846;
    const float rest = L / (NODES - 1);
    float phi = 0.3f, t = 0.0f;
    Kinematics kin = makeKin({15.0f, 8.0f}, phi, V_MIN, R_MAX);
    SharkBody body = seedSharkBody(kin, params);
    for (int step = 0; step < 1200; ++step) {
        CAPTURE(step);
        float dt = uneven[step % 8];
        t += dt;
        float v = V_MIN + (V_MAX - V_MIN) * 0.5f * (1.0f - std::cos(0.9f * t));
        float r = R_MAX * std::sin(1.7f * t + 1.5f);
        Vector2 pos = {kin.pos.x + kin.heading.x * kin.v * dt,
                       kin.pos.y + kin.heading.y * kin.v * dt};
        phi += kin.r * dt;
        kin = makeKin(pos, phi, v, r);

        double expected = body.phase + pi * v * dt;
        body = updateSharkBody(body, kin, params, dt);
        CHECK(std::abs(std::remainder(body.phase - expected, 2.0 * pi)) <= 1e-6);
        REQUIRE(body.spine.size() == NODES);
        for (int i = 0; i + 1 < NODES; ++i) {
            CAPTURE(i);
            float d = std::hypot(body.spine[i + 1].x - body.spine[i].x,
                                 body.spine[i + 1].y - body.spine[i].y);
            CHECK(std::abs(d - rest) <= 0.001f * rest);
        }
    }
}
