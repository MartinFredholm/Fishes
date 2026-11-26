#include "doctest/doctest.h"

#include "motion.h"
#include <cmath>

namespace {

constexpr float TOL = 1e-6f;

constexpr MotionParams LAG_PARAMS = {.v_min = 0.5f,
                                     .v_cruise = 1.0f,
                                     .v_max = 2.5f,
                                     .r_max = 0.8f,
                                     .T_v = 1.0f,
                                     .T_r = 0.5f,
                                     .K_phi = 4.0f,
                                     .tau = 0.3f,
                                     .wall_clearance = 0.0f};

} // namespace

TEST_CASE("the world is 30 by 16 m") {
    CHECK(WORLD_WIDTH == doctest::Approx(30.0f));
    CHECK(WORLD_HEIGHT == doctest::Approx(16.0f));
}

TEST_CASE("integrate moves along the old heading and lags speed and turn "
          "rate toward the commands") {
    Kinematics kin{{1.0f, 2.0f}, 0.0f, 1.0f, 0.5f, {1.0f, 0.0f}};
    Commands cmd{.v_cmd = 2.0f, .r_cmd = 1.0f};

    Kinematics next = integrate(kin, cmd, LAG_PARAMS, 0.1f);

    CHECK(next.pos.x == doctest::Approx(1.1f).epsilon(TOL));
    CHECK(next.pos.y == doctest::Approx(2.0f).epsilon(TOL));
    CHECK(next.phi == doctest::Approx(0.05f).epsilon(TOL));
    CHECK(next.v == doctest::Approx(1.1f).epsilon(TOL));
    CHECK(next.r == doctest::Approx(0.6f).epsilon(TOL));
    CHECK(next.heading.x == doctest::Approx(std::cos(0.05f)).epsilon(TOL));
    CHECK(next.heading.y == doctest::Approx(-std::sin(0.05f)).epsilon(TOL));
}
