#include <AuxImageType.hpp>
#include <RGBD-PixelTo3D.hpp>
#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include "TestsMatches.hpp"

TEST_CASE("Converting 1,2,3 to 3D") {
	RGBD::CameraIntrinsics camIntrisics(1, 2, 3, 4, -0.1);
	PixelTo3D p(camIntrisics);
	auto conversion = p.convertTo3D(1, 2, 3);
	//std::cout << conversion << "\n";
	REQUIRE_THAT(*conversion, EqualsPoseCoords(RGBD::PoseCoords(-6.3, -3, 3)));
	delete conversion;
}