#include "doctest/doctest.h"

#include "shark_body.h"
#include <cmath>

namespace {

BodyParams flatParams() {
    return {.length = 3.0f,
            .node_count = 24,
            .wave_amplitude = 0.0f,
            .wave_number = 5.3f,
            .strouhal = 0.3f,
            .length_passes = 1};
}

Kinematics makeKin(Vector2 pos, float phi, float v, float r) {
    return {pos, phi, v, r, {std::cos(phi), -std::sin(phi)}};
}

} // namespace

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
