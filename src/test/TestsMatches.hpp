#pragma once

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_templated.hpp>
#include <RGBD-PixelTo3D.hpp>
#include <catch2/catch_approx.hpp>

struct EqualsPoseCoordsMatcher : Catch::Matchers::MatcherGenericBase {
    EqualsPoseCoordsMatcher(RGBD::PoseCoords const& poseCoords) : pc(poseCoords) {}

    bool match(RGBD::PoseCoords const& other) const {
        return pc.x == Catch::Approx(other.x) && pc.y == Catch::Approx(other.y) && pc.z == Catch::Approx(other.z);
    }

    std::string describe() const override {
        return "Equals: " + pc.to_string();
    }

private:
    RGBD::PoseCoords const& pc;
};

inline auto EqualsPoseCoords(const RGBD::PoseCoords& pc) -> EqualsPoseCoordsMatcher {
    return EqualsPoseCoordsMatcher{pc};
}