#include "doctest/doctest.h"

#include "shark_body.h"
#include <cmath>
#include <vector>

namespace {

constexpr float L = 3.0f;
constexpr double PI_R = 3.14159265358979323846;

BodyParams reviewParams(int nodes, int passes) {
    return {.length = L,
            .node_count = nodes,
            .wave_amplitude = 0.1f,
            .wave_number = 5.3f,
            .strouhal = 0.3f,
            .length_passes = passes};
}

Kinematics kinHeading(Vector2 pos, float phi, float v) {
    return {pos, phi, v, 0.0f, {std::cos(phi), -std::sin(phi)}};
}

Kinematics advanceStraight(const Kinematics &kin, float dt) {
    return kinHeading({kin.pos.x + kin.heading.x * kin.v * dt,
                       kin.pos.y + kin.heading.y * kin.v * dt},
                      kin.phi, kin.v);
}

double lateral(Vector2 p, Vector2 origin, Vector2 fwd) {
    return (double)(p.x - origin.x) * fwd.y - (double)(p.y - origin.y) * fwd.x;
}

std::vector<double> lateralOffsets(float phase, int nodes) {
    const BodyParams params = reviewParams(nodes, 0);
    Kinematics kin = kinHeading({3.0f, 1.0f}, 0.0f, 0.0f);
    SharkBody body = seedSharkBody(kin, params);
    body.phase = phase;
    SharkBody next = updateSharkBody(body, kin, params, 0.0f);
    std::vector<double> h(nodes);
    for (int i = 0; i < nodes; ++i)
        h[i] = lateral(next.spine[i], kin.pos, kin.heading);
    return h;
}

} // namespace

TEST_CASE("review 03 / R8: amp(s) passes through the six points and has no "
          "slope jump at them") {
    const int nodes = 301;
    const double knots[] = {0.2, 0.21, 0.23, 0.4, 0.88, 1.0};
    std::vector<double> h_sin = lateralOffsets(0.7f, nodes);
    std::vector<double> h_cos = lateralOffsets(0.7f + (float)PI_R / 2.0f, nodes);
    std::vector<double> amp(nodes);
    for (int i = 0; i < nodes; ++i)
        amp[i] = std::hypot(h_sin[i], h_cos[i]) / (0.1 * L);

    const double ds = 1.0 / (nodes - 1);
    for (int k = 0; k < 6; ++k) {
        CAPTURE(k);
        int i = k * 60;
        CHECK(amp[i] == doctest::Approx(knots[k]).epsilon(1e-3));
        if (k == 0 || k == 5)
            continue;
        double left = (amp[i] - amp[i - 1]) / ds;
        double right = (amp[i + 1] - amp[i]) / ds;
        CHECK(std::abs(right - left) < 0.3);
    }
}

TEST_CASE("review 03 / design: the drawn nose sways about kin.pos by "
          "A*L*amp(0) = 0.06 m" *
          doctest::skip() * doctest::description("moves to issue 06")) {
    const BodyParams params = reviewParams(24, 1);
    const float dt = 1.0f / 60.0f;
    Kinematics kin = kinHeading({5.0f, 8.0f}, 0.5f, 1.0f);
    SharkBody body = seedSharkBody(kin, params);
    double peak = 0.0;
    for (int step = 0; step < 600; ++step) {
        kin = advanceStraight(kin, dt);
        body = updateSharkBody(body, kin, params, dt);
        double sway = std::hypot(body.spine[0].x - kin.pos.x,
                                 body.spine[0].y - kin.pos.y);
        CHECK(sway <= 0.06 + 1e-4);
        peak = std::fmax(peak, sway);
    }
    CHECK(peak == doctest::Approx(0.06).epsilon(0.02));
}

TEST_CASE("review 03 / R11 + R12: tail-tip peak with uneven dt is 0.30 m "
          "+/- 20 %") {
    const BodyParams params = reviewParams(24, 1);
    const float uneven[] = {1.0f / 30.0f, 0.007f, 1.0f / 144.0f, 0.019f,
                            0.001f};
    Kinematics kin = kinHeading({2.0f, 3.0f}, -1.1f, 1.0f);
    const Vector2 origin = kin.pos, fwd = kin.heading;
    SharkBody body = seedSharkBody(kin, params);
    float t = 0.0f;
    int step = 0;
    double peak = 0.0;
    while (t < L + 2.0f) {
        float dt = uneven[step++ % 5];
        t += dt;
        kin = advanceStraight(kin, dt);
        body = updateSharkBody(body, kin, params, dt);
        if (t >= L)
            peak = std::fmax(peak, std::abs(lateral(body.spine.back(), origin, fwd)));
    }
    CHECK(peak == doctest::Approx(0.30).epsilon(0.20));
}
