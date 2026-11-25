#include "doctest/doctest.h"

#include "shark_body.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace {

constexpr float L = 3.0f;
constexpr int NODES = 24;
constexpr float S_P = 1.0f / 3.0f;
constexpr double TOL = 1e-3;       // m
constexpr double FLOAT_TOL = 1e-5; // m, float rounding over a few metres
constexpr float V_MIN = 0.5f, V_CRUISE = 1.0f, V_MAX = 2.5f, R_MAX = 0.8f;
constexpr double NEED = (1.0 - S_P) * L;

BodyParams pivotBody(float amplitude) {
    return {.length = L,
            .node_count = NODES,
            .wave_amplitude = amplitude,
            .wave_number = 5.3f,
            .strouhal = 0.3f,
            .length_passes = 1,
            .pivot = S_P};
}

Kinematics makeKin(Vector2 pos, double phi, float v, float r) {
    return {pos, (float)phi, v, r,
            {(float)std::cos(phi), (float)-std::sin(phi)}};
}

double nodeS(int i) { return (double)i / (NODES - 1); }

struct Point {
    double x, y;
};

double dist(Vector2 a, Point b) { return std::hypot(a.x - b.x, a.y - b.y); }
double dist(Vector2 a, Vector2 b) { return std::hypot(a.x - b.x, a.y - b.y); }

double polylineLength(const std::vector<Vector2> &path, size_t from) {
    double length = 0.0;
    for (size_t k = from + 1; k < path.size(); ++k)
        length += dist(path[k], path[k - 1]);
    return length;
}

// pivot_path layout: oldest point first.
double coverage(const SharkBody &body, const Kinematics &kin, size_t from = 0) {
    if (from >= body.pivot_path.size())
        return 0.0;
    return polylineLength(body.pivot_path, from) +
           dist(body.pivot_path.back(), kin.pos);
}

// y is down, so heading = (cos phi, -sin phi).
Point arcAhead(double x, double y, double phi, double kappa, double ahead) {
    int n = std::max(1, (int)std::ceil(ahead / 1e-3));
    double ds = ahead / n;
    for (int k = 0; k < n; ++k) {
        double mid = phi + kappa * ds / 2;
        x += std::cos(mid) * ds;
        y -= std::sin(mid) * ds;
        phi += kappa * ds;
    }
    return {x, y};
}

class Swimmer {
  public:
    Swimmer(Point start, double phi, float v, float r)
        : x_(start.x), y_(start.y), phi_(phi), v_(v), r_(r) {
        // The straight lead-in is the seed's straight pivot path.
        add({start.x - std::cos(phi) * 2 * L, start.y + std::sin(phi) * 2 * L});
        add(start);
    }

    Kinematics kin() const { return makeKin({(float)x_, (float)y_}, phi_, v_, r_); }

    void swim(float dt, float v_end, float r_end) {
        const int substeps = 64;
        double h = (double)dt / substeps;
        for (int k = 0; k < substeps; ++k) {
            double f = (k + 0.5) / substeps;
            double v = v_ + (v_end - v_) * f, r = r_ + (r_end - r_) * f;
            double mid = phi_ + r * h / 2;
            x_ += std::cos(mid) * v * h;
            y_ -= std::sin(mid) * v * h;
            phi_ += r * h;
            add({x_, y_});
        }
        v_ = v_end;
        r_ = r_end;
    }

    Point behind(double d) const {
        double target = arc_.back() - d;
        REQUIRE(target >= arc_.front());
        size_t k = std::upper_bound(arc_.begin(), arc_.end(), target) - arc_.begin();
        k = std::clamp<size_t>(k, 1, arc_.size() - 1);
        double t = (target - arc_[k - 1]) / (arc_[k] - arc_[k - 1]);
        return {pts_[k - 1].x + t * (pts_[k].x - pts_[k - 1].x),
                pts_[k - 1].y + t * (pts_[k].y - pts_[k - 1].y)};
    }

    Point ahead(double d) const {
        double kappa = v_ > 0 ? r_ / v_ : 0.0;
        return arcAhead(x_, y_, phi_, kappa, d);
    }

  private:
    void add(Point p) {
        if (!pts_.empty()) {
            double step = std::hypot(p.x - pts_.back().x, p.y - pts_.back().y);
            if (step == 0.0)
                return;
            arc_.push_back(arc_.back() + step);
        } else {
            arc_.push_back(0.0);
        }
        pts_.push_back(p);
    }

    double x_, y_, phi_;
    float v_, r_;
    std::vector<Point> pts_;
    std::vector<double> arc_;
};

Point expectedNode(const Swimmer &swimmer, int i) {
    double s = nodeS(i);
    return s >= S_P ? swimmer.behind((s - S_P) * L) : swimmer.ahead((S_P - s) * L);
}

void checkOnPathAndHeadArc(const SharkBody &body, const Swimmer &swimmer) {
    REQUIRE(body.spine.size() == NODES);
    for (int i = 0; i < NODES; ++i) {
        CAPTURE(i);
        CHECK(dist(body.spine[i], expectedNode(swimmer, i)) < TOL);
    }
}

Kinematics straightStep(const Kinematics &kin, float dt) {
    return makeKin({kin.pos.x + kin.heading.x * kin.v * dt,
                    kin.pos.y + kin.heading.y * kin.v * dt},
                   kin.phi, kin.v, kin.r);
}

} // namespace

TEST_CASE("R6: the seed lays a straight pivot path behind kin.pos, and the "
          "first update at cruise moves no node more than 0.05 m") {
    const float dt = 1.0f / 60.0f;
    for (float phi : {0.0f, 2.3f, -0.9f}) {
        CAPTURE(phi);
        const Kinematics kin = makeKin({7.0f, 4.0f}, phi, V_CRUISE, 0.0f);
        SharkBody seed = seedSharkBody(kin, pivotBody(0.1f));

        REQUIRE(seed.pivot_path.size() >= 2);
        CHECK(coverage(seed, kin) >= NEED - FLOAT_TOL);
        for (Vector2 p : seed.pivot_path) {
            double along = (p.x - kin.pos.x) * kin.heading.x +
                           (p.y - kin.pos.y) * kin.heading.y;
            double across = (p.x - kin.pos.x) * kin.heading.y -
                            (p.y - kin.pos.y) * kin.heading.x;
            CHECK(along <= FLOAT_TOL);
            CHECK(std::abs(across) <= FLOAT_TOL);
        }

        SharkBody first =
            updateSharkBody(seed, straightStep(kin, dt), pivotBody(0.1f), dt);
        REQUIRE(seed.spine.size() == NODES);
        REQUIRE(first.spine.size() == NODES);
        for (int i = 0; i < NODES; ++i) {
            CAPTURE(i);
            CHECK(dist(first.spine[i], seed.spine[i]) <= 0.05);
        }
    }
}

TEST_CASE("R7: zero amplitude, nodes behind the pivot on its track at "
          "(s_i - s_p) L, head nodes on the head arc at (s_p - s_i) L") {
    const BodyParams params = pivotBody(0.0f);
    const float dt = 1.0f / 60.0f;
    struct Case {
        std::string name;
        float (*v)(float t);
        float (*r)(float t);
    };
    const Case cases[] = {
        {"straight", [](float) { return V_CRUISE; }, [](float) { return 0.0f; }},
        {"v_min, +r_max", [](float) { return V_MIN; }, [](float) { return R_MAX; }},
        {"v_min, -r_max", [](float) { return V_MIN; }, [](float) { return -R_MAX; }},
        {"v_max, +r_max", [](float) { return V_MAX; }, [](float) { return R_MAX; }},
        {"v_max, -r_max", [](float) { return V_MAX; }, [](float) { return -R_MAX; }},
        {"varying r and v",
         [](float t) { return 1.5f + 1.0f * std::sin(0.7f * t); },
         [](float t) { return R_MAX * std::sin(1.3f * t + 0.4f); }},
    };
    for (const Case &c : cases) {
        for (double phi : {0.0, 2.2, -1.3}) {
            CAPTURE(c.name);
            CAPTURE(phi);
            Swimmer swimmer({12.0, 7.0}, phi, c.v(0.0f), c.r(0.0f));
            SharkBody body = seedSharkBody(swimmer.kin(), params);
            checkOnPathAndHeadArc(body, swimmer);
            float t = 0.0f;
            for (int step = 0; step < 480; ++step) {
                CAPTURE(step);
                t += dt;
                swimmer.swim(dt, c.v(t), c.r(t));
                body = updateSharkBody(body, swimmer.kin(), params, dt);
                checkOnPathAndHeadArc(body, swimmer);
            }
        }
    }
}

TEST_CASE("R12: uneven dt (all <= 1/30 s, including 0) while turning keeps "
          "R7, R9 and R10") {
    const float uneven[] = {1.0f / 30.0f, 0.0f,   0.004f, 1.0f / 144.0f,
                            0.021f,       0.0005f, 0.0f,  1.0f / 33.0f};
    auto v_at = [](float t) { return 1.5f + 1.0f * std::sin(0.9f * t); };
    auto r_at = [](float t) { return R_MAX * std::sin(1.7f * t + 1.5f); };

    SUBCASE("R7, zero amplitude") {
        const BodyParams params = pivotBody(0.0f);
        Swimmer swimmer({15.0, 8.0}, 0.3, v_at(0.0f), r_at(0.0f));
        SharkBody body = seedSharkBody(swimmer.kin(), params);
        float t = 0.0f;
        for (int step = 0; step < 1200; ++step) {
            CAPTURE(step);
            float dt = uneven[step % 8];
            t += dt;
            swimmer.swim(dt, v_at(t), r_at(t));
            body = updateSharkBody(body, swimmer.kin(), params, dt);
            checkOnPathAndHeadArc(body, swimmer);
        }
    }
    SUBCASE("R9 and R10, wave on") {
        const BodyParams params = pivotBody(0.1f);
        const double pi = 3.14159265358979323846;
        const double rest = L / (NODES - 1);
        Swimmer swimmer({15.0, 8.0}, 0.3, v_at(0.0f), r_at(0.0f));
        SharkBody body = seedSharkBody(swimmer.kin(), params);
        float t = 0.0f;
        for (int step = 0; step < 1200; ++step) {
            CAPTURE(step);
            float dt = uneven[step % 8];
            t += dt;
            swimmer.swim(dt, v_at(t), r_at(t));
            double expected = body.phase + pi * v_at(t) * dt;
            body = updateSharkBody(body, swimmer.kin(), params, dt);
            CHECK(std::abs(std::remainder(body.phase - expected, 2.0 * pi)) <= 1e-6);
            REQUIRE(body.spine.size() == NODES);
            for (int i = 0; i + 1 < NODES; ++i) {
                CAPTURE(i);
                CHECK(std::abs(dist(body.spine[i + 1], body.spine[i]) - rest) <=
                      0.001 * rest);
            }
        }
    }
}

TEST_CASE("R20: v = 0 gives a straight head along kin.heading at "
          "(s_p - s_i) L") {
    const BodyParams params = pivotBody(0.0f);
    for (float r : {0.0f, R_MAX, -R_MAX}) {
        CAPTURE(r);
        const double phi = 1.9;
        const Kinematics kin = makeKin({6.0f, 9.0f}, phi, 0.0f, r);
        SharkBody body =
            updateSharkBody(seedSharkBody(kin, params), kin, params, 1.0f / 60.0f);
        REQUIRE(body.spine.size() == NODES);
        for (int i = 0; nodeS(i) < S_P; ++i) {
            CAPTURE(i);
            double ahead = (S_P - nodeS(i)) * L;
            Point expected = {6.0 + std::cos(phi) * ahead, 9.0 - std::sin(phi) * ahead};
            CHECK(dist(body.spine[i], expected) < TOL);
        }
    }
}

TEST_CASE("R20: an update with v = 0 and kin.pos unchanged leaves the nodes "
          "behind the pivot where they were") {
    const BodyParams params = pivotBody(0.0f);
    const float dt = 1.0f / 60.0f;
    for (float r_stop : {0.0f, R_MAX, -R_MAX}) {
        for (float dt_stop : {0.0f, dt}) {
            CAPTURE(r_stop);
            CAPTURE(dt_stop);
            Swimmer swimmer({10.0, 6.0}, 0.7, V_MIN, R_MAX);
            SharkBody body = seedSharkBody(swimmer.kin(), params);
            for (int step = 0; step < 240; ++step) {
                swimmer.swim(dt, V_MIN, step < 120 ? R_MAX : -R_MAX);
                body = updateSharkBody(body, swimmer.kin(), params, dt);
            }
            Kinematics stopped = swimmer.kin();
            stopped.v = 0.0f;
            stopped.r = r_stop;
            SharkBody still = updateSharkBody(body, stopped, params, dt_stop);
            REQUIRE(still.spine.size() == NODES);
            for (int i = 0; i < NODES; ++i) {
                if (nodeS(i) < S_P)
                    continue;
                CAPTURE(i);
                CHECK(dist(still.spine[i], body.spine[i]) < TOL);
            }
        }
    }
}

TEST_CASE("R20: v from 0 up to v_min at r = +-r_max keeps every coordinate "
          "finite, with the head ahead of the pivot") {
    const float speeds[] = {0.0f,  std::numeric_limits<float>::denorm_min(),
                            1e-38f, 1e-30f, 1e-12f, 1e-6f, 1e-3f, 0.05f,
                            0.25f,  V_MIN};
    const float dt = 1.0f / 60.0f;
    for (float amplitude : {0.0f, 0.1f}) {
        const BodyParams params = pivotBody(amplitude);
        for (float v : speeds) {
            for (float r : {R_MAX, -R_MAX}) {
                CAPTURE(amplitude);
                CAPTURE(v);
                CAPTURE(r);
                Swimmer swimmer({6.0, 9.0}, 0.6, V_MIN, r);
                SharkBody body = seedSharkBody(swimmer.kin(), params);
                for (int step = 0; step < 60; ++step) {
                    swimmer.swim(dt, step < 30 ? V_MIN : v, r);
                    body = updateSharkBody(body, swimmer.kin(), params, dt);
                }
                REQUIRE(body.spine.size() == NODES);
                for (Vector2 p : body.spine) {
                    CHECK(std::isfinite(p.x));
                    CHECK(std::isfinite(p.y));
                }
            }
        }
    }
}

TEST_CASE("R21: on long swims the path covers (1 - s_p) L behind kin.pos, "
          "keeps no more than that requires and stays bounded") {
    const float dt = 1.0f / 60.0f;
    for (float amplitude : {0.0f, 0.1f}) {
        for (float v : {V_MIN, V_CRUISE, V_MAX}) {
            for (float r : {0.0f, R_MAX}) {
                CAPTURE(amplitude);
                CAPTURE(v);
                CAPTURE(r);
                const BodyParams params = pivotBody(amplitude);
                const size_t bound = (size_t)std::ceil(NEED / (v * dt)) + 2;
                Swimmer swimmer({14.0, 8.0}, -0.4, v, r);
                SharkBody body = seedSharkBody(swimmer.kin(), params);
                for (int step = 0; step < 6000; ++step) {
                    CAPTURE(step);
                    swimmer.swim(dt, v, step % 600 < 300 ? r : -r);
                    const Kinematics kin = swimmer.kin();
                    body = updateSharkBody(body, kin, params, dt);
                    REQUIRE(body.pivot_path.size() >= 2);
                    CHECK(coverage(body, kin) >= NEED - FLOAT_TOL);
                    CHECK(coverage(body, kin, 1) < NEED + FLOAT_TOL);
                    CHECK(body.pivot_path.size() <= bound);
                }
            }
        }
    }
}

TEST_CASE("R22: same pivot path and phase but different r on the last update "
          "give the same spine on the next update") {
    const BodyParams params = pivotBody(0.1f);
    const float dt = 1.0f / 60.0f;
    Swimmer swimmer({9.0, 5.0}, 1.1, 1.8f, 0.3f);
    SharkBody before = seedSharkBody(swimmer.kin(), params);
    for (int step = 0; step < 180; ++step) {
        swimmer.swim(dt, 1.8f, 0.3f);
        before = updateSharkBody(before, swimmer.kin(), params, dt);
    }
    swimmer.swim(dt, 1.8f, 0.3f);
    Kinematics turning_left = swimmer.kin(), turning_right = swimmer.kin();
    turning_left.r = R_MAX;
    turning_right.r = -R_MAX;
    SharkBody a = updateSharkBody(before, turning_left, params, dt);
    SharkBody b = updateSharkBody(before, turning_right, params, dt);
    REQUIRE(a.pivot_path.size() == b.pivot_path.size());
    for (size_t k = 0; k < a.pivot_path.size(); ++k) {
        CHECK(a.pivot_path[k].x == b.pivot_path[k].x);
        CHECK(a.pivot_path[k].y == b.pivot_path[k].y);
    }
    REQUIRE(a.phase == b.phase);
    REQUIRE(dist(a.spine[0], b.spine[0]) > 0.1);

    swimmer.swim(dt, 1.8f, -0.5f);
    SharkBody next_a = updateSharkBody(a, swimmer.kin(), params, dt);
    SharkBody next_b = updateSharkBody(b, swimmer.kin(), params, dt);
    REQUIRE(next_a.spine.size() == NODES);
    REQUIRE(next_b.spine.size() == NODES);
    for (int i = 0; i < NODES; ++i) {
        CAPTURE(i);
        CHECK(dist(next_a.spine[i], next_b.spine[i]) < 1e-4);
    }
}

TEST_CASE("R11: with the pivot at s_p, straight at cruise, the tail tip's peak "
          "offset over a beat is 0.30 m +/- 20 %") {
    const BodyParams params = pivotBody(0.1f);
    const float dt = 1.0f / 60.0f;
    const float beat_period = 2.0f / V_CRUISE;
    Kinematics kin = makeKin({3.0f, 8.0f}, -0.4, V_CRUISE, 0.0f);
    const Vector2 origin = kin.pos, fwd = kin.heading;
    SharkBody body = seedSharkBody(kin, params);
    float t = 0.0f;
    for (; t < L / V_CRUISE; t += dt) {
        kin = straightStep(kin, dt);
        body = updateSharkBody(body, kin, params, dt);
    }
    double peak = 0.0;
    for (float end = t + beat_period; t < end; t += dt) {
        kin = straightStep(kin, dt);
        body = updateSharkBody(body, kin, params, dt);
        Vector2 tip = body.spine.back();
        peak = std::fmax(peak, std::abs((tip.x - origin.x) * fwd.y -
                                        (tip.y - origin.y) * fwd.x));
    }
    CHECK(peak == doctest::Approx(0.30).epsilon(0.20));
}
