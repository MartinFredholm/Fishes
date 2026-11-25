#include "doctest/doctest.h"

#include "shark_body.h"
#include <cmath>

TEST_CASE("review 01: default heading of an aggregate-initialised Kinematics "
          "matches phi (randomStart relies on it)") {
    Kinematics k{{1.0f, 2.0f}, 1.1f, 1.0f, 0.0f};
    CHECK(k.heading.x == doctest::Approx(std::cos(1.1f)));
    CHECK(k.heading.y == doctest::Approx(-std::sin(1.1f)));
}
