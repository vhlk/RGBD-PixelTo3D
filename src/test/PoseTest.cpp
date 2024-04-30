#include <AuxImageType.hpp>
#include <RGBD-Pose.hpp>
#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include "TestsMatches.hpp"
#include <opencv2/opencv.hpp>
#include <pybind11/pybind11.h>
#include <catch2/catch_session.hpp>

constexpr bool RUN_BENCHMARKS = false;

namespace py = pybind11;

// specific sensor intrinsic values which was used for capturing images
const Eigen::Vector4d intrinsics(579.609, 579.609, 317.791, 247.62);
const int img_width = 640;
const int img_height = 480;
RGBD::CameraIntrinsics ci(intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3]);
std::string test_dir = TESTS_DIR;
std::string rgbImagePath = test_dir + "/rgbImage.png";
std::string depthImagePath = test_dir + "/depthImage.png";
cv::Mat rgbImage = cv::imread(rgbImagePath, cv::ImreadModes::IMREAD_COLOR);
cv::Mat depthImage = cv::imread(depthImagePath, cv::ImreadModes::IMREAD_ANYDEPTH);

// mostly for making sure py::module_ is properly destructed
struct PoseModule {
	py::module_ mp;
	Pose* p;

	PoseModule(py::module_ mp, Pose* p) {
		this->mp = mp;
		this->p = p;
	}
};

PoseModule* pm;

TEST_CASE("Extracting pose 3D with update") {
	const auto optCoordsUpd = pm->p->update((RGBD::RGB888Pixel*)rgbImage.data, (RGBD::DepthPixel*)depthImage.data);

	REQUIRE(optCoordsUpd.has_value() == true);

	const auto& coords = optCoordsUpd.value();
	// checking random landmarks

	// head is above shoulders, hips, knees and feets
	auto head = coords[0];
	auto shoulder_left = coords[12];
	auto shoulder_right = coords[11];
	auto hip_left = coords[24];
	auto hip_right = coords[23];
	auto knee_left = coords[26];
	auto knee_right = coords[25];
	auto feet_left = coords[28];
	auto feet_right = coords[27];

	// values increse from top to bottom
	REQUIRE(head->pc->y < shoulder_left->pc->y);
	REQUIRE(head->pc->y < shoulder_right->pc->y);
	REQUIRE(head->pc->y < hip_left->pc->y);
	REQUIRE(head->pc->y < hip_right->pc->y);
	REQUIRE(head->pc->y < knee_left->pc->y);
	REQUIRE(head->pc->y < knee_right->pc->y);
	REQUIRE(head->pc->y < feet_left->pc->y);
	REQUIRE(head->pc->y < feet_right->pc->y);
}

TEST_CASE("Extracting pose 3D with update 2D and 3D") {
	const auto optCoordsUpd2D3D = pm->p->update2Dand3D((RGBD::RGB888Pixel*)rgbImage.data, (RGBD::DepthPixel*)depthImage.data);

	REQUIRE(optCoordsUpd2D3D.has_value() == true);

	const auto& coords2D3D = optCoordsUpd2D3D.value();

	// checking random landmarks

	// head is above shoulders, hips, knees and feets
	auto head2D = coords2D3D.first[0];
	auto shoulder_left2D = coords2D3D.first[12];
	auto shoulder_right2D = coords2D3D.first[11];
	auto hip_left2D = coords2D3D.first[24];
	auto hip_right2D = coords2D3D.first[23];
	auto knee_left2D = coords2D3D.first[26];
	auto knee_right2D = coords2D3D.first[25];
	auto feet_left2D = coords2D3D.first[28];
	auto feet_right2D = coords2D3D.first[27];

	// values increse from top to bottom
	REQUIRE(head2D->pc->y < shoulder_left2D->pc->y);
	REQUIRE(head2D->pc->y < shoulder_right2D->pc->y);
	REQUIRE(head2D->pc->y < hip_left2D->pc->y);
	REQUIRE(head2D->pc->y < hip_right2D->pc->y);
	REQUIRE(head2D->pc->y < knee_left2D->pc->y);
	REQUIRE(head2D->pc->y < knee_right2D->pc->y);
	REQUIRE(head2D->pc->y < feet_left2D->pc->y);
	REQUIRE(head2D->pc->y < feet_right2D->pc->y);

	// head is above shoulders, hips, knees and feets
	auto head3D = coords2D3D.second[0];
	auto shoulder_left3D = coords2D3D.second[12];
	auto shoulder_right3D = coords2D3D.second[11];
	auto hip_left3D = coords2D3D.second[24];
	auto hip_right3D = coords2D3D.second[23];
	auto knee_left3D = coords2D3D.second[26];
	auto knee_right3D = coords2D3D.second[25];
	auto feet_left3D = coords2D3D.second[28];
	auto feet_right3D = coords2D3D.second[27];

	// values increse from top to bottom
	REQUIRE(head3D->pc->y < shoulder_left3D->pc->y);
	REQUIRE(head3D->pc->y < shoulder_right3D->pc->y);
	REQUIRE(head3D->pc->y < hip_left3D->pc->y);
	REQUIRE(head3D->pc->y < hip_right3D->pc->y);
	REQUIRE(head3D->pc->y < knee_left3D->pc->y);
	REQUIRE(head3D->pc->y < knee_right3D->pc->y);
	REQUIRE(head3D->pc->y < feet_left3D->pc->y);
	REQUIRE(head3D->pc->y < feet_right3D->pc->y);
}

TEST_CASE("Extracting pose 3D with update RGBD and Mediapipe") {
	const auto optCoordsUpdRGBDMP = pm->p->RGBDandMediaPipeUpdate((RGBD::RGB888Pixel*)rgbImage.data, (RGBD::DepthPixel*)depthImage.data);

	REQUIRE(optCoordsUpdRGBDMP.has_value() == true);

	const auto& coords2D3D = optCoordsUpdRGBDMP.value();

	// checking random landmarks

	// head is above shoulders, hips, knees and feets
	auto headRGBD = coords2D3D.first[0];
	auto shoulder_leftRGBD = coords2D3D.first[12];
	auto shoulder_rightRGBD = coords2D3D.first[11];
	auto hip_leftRGBD = coords2D3D.first[24];
	auto hip_rightRGBD = coords2D3D.first[23];
	auto knee_leftRGBD = coords2D3D.first[26];
	auto knee_rightRGBD = coords2D3D.first[25];
	auto feet_leftRGBD = coords2D3D.first[28];
	auto feet_rightRGBD = coords2D3D.first[27];

	// values increse from top to bottom
	REQUIRE(headRGBD->pc->y < shoulder_leftRGBD->pc->y);
	REQUIRE(headRGBD->pc->y < shoulder_rightRGBD->pc->y);
	REQUIRE(headRGBD->pc->y < hip_leftRGBD->pc->y);
	REQUIRE(headRGBD->pc->y < hip_rightRGBD->pc->y);
	REQUIRE(headRGBD->pc->y < knee_leftRGBD->pc->y);
	REQUIRE(headRGBD->pc->y < knee_rightRGBD->pc->y);
	REQUIRE(headRGBD->pc->y < feet_leftRGBD->pc->y);
	REQUIRE(headRGBD->pc->y < feet_rightRGBD->pc->y);

	// head is above shoulders, hips, knees and feets
	auto headMP = coords2D3D.second[0];
	auto shoulder_leftMP = coords2D3D.second[12];
	auto shoulder_rightMP = coords2D3D.second[11];
	auto hip_leftMP = coords2D3D.second[24];
	auto hip_rightMP = coords2D3D.second[23];
	auto knee_leftMP = coords2D3D.second[26];
	auto knee_rightMP = coords2D3D.second[25];
	auto feet_leftMP = coords2D3D.second[28];
	auto feet_rightMP = coords2D3D.second[27];

	// values increse from top to bottom
	REQUIRE(headMP->pc->y < shoulder_leftMP->pc->y);
	REQUIRE(headMP->pc->y < shoulder_rightMP->pc->y);
	REQUIRE(headMP->pc->y < hip_leftMP->pc->y);
	REQUIRE(headMP->pc->y < hip_rightMP->pc->y);
	REQUIRE(headMP->pc->y < knee_leftMP->pc->y);
	REQUIRE(headMP->pc->y < knee_rightMP->pc->y);
	REQUIRE(headMP->pc->y < feet_leftMP->pc->y);
	REQUIRE(headMP->pc->y < feet_rightMP->pc->y);
}

TEST_CASE("Benchmark") {
	if (!RUN_BENCHMARKS) return;

	constexpr int NUM_OF_TESTS = 1'000;

	size_t sum = 0;
	for (int i = 0; i < NUM_OF_TESTS; i++) {
		auto start = std::chrono::high_resolution_clock::now();
		const auto optCoordsUpd = pm->p->update((RGBD::RGB888Pixel*)rgbImage.data, (RGBD::DepthPixel*)depthImage.data);
		auto elapsed = std::chrono::high_resolution_clock::now() - start;
		sum += std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
	}
	size_t average = sum / NUM_OF_TESTS;
	std::cout << "Update: " << average << " ms" << '\n';

	sum = 0;
	for (int i = 0; i < NUM_OF_TESTS; i++) {
		auto start = std::chrono::high_resolution_clock::now();
		const auto optCoordsUpd = pm->p->update2Dand3D((RGBD::RGB888Pixel*)rgbImage.data, (RGBD::DepthPixel*)depthImage.data);
		auto elapsed = std::chrono::high_resolution_clock::now() - start;
		sum += std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
	}
	average = sum / NUM_OF_TESTS;
	std::cout << "Update2Dand3D: " << average << " ms" << '\n';

	sum = 0;
	for (int i = 0; i < NUM_OF_TESTS; i++) {
		auto start = std::chrono::high_resolution_clock::now();
		const auto optCoordsUpd = pm->p->RGBDandMediaPipeUpdate((RGBD::RGB888Pixel*)rgbImage.data, (RGBD::DepthPixel*)depthImage.data);
		auto elapsed = std::chrono::high_resolution_clock::now() - start;
		sum += std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
	}
	average = sum / NUM_OF_TESTS;
	std::cout << "RGBDandMediaPipeUpdate: " << average << " ms" << '\n';	
}

TEST_CASE("Construction and destruction of Pose") {

	constexpr int NUM_OF_CALLS = 10;
	for (int i = 0; i < NUM_OF_CALLS; i++) {
		Pose p1(ci, img_width, img_height, pm->mp);
	}
}

int main(int argc, char* argv[]) {
	int result;

	py::scoped_interpreter py_guard{};
	{
		py::module_ mp = py::module_::import("mediapipe");
		Pose* p = new Pose(ci, img_width, img_height, mp);
		pm = new PoseModule(mp, p);

		result = Catch::Session().run(argc, argv);

		delete pm;
		delete p;
	}

	return result;
}