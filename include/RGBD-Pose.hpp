#pragma once

#define USE_MEDIAPIPE_PYBIND 1

#include "RGBD-PixelTo3D.hpp"
#ifndef USE_MEDIAPIPE_PYBIND
#ifdef USE_REDIS
#include <hiredis.h>
#else
#include <SocketAux.hpp>
#endif // USE_REDIS
#endif // USE_MEDIAPIPE_PYBIND
#include <tuple>
#include <optional>
#include <array>
#include "AuxImageType.hpp"
#include <utility> // pair

#include <pybind11/pybind11.h>
#include <pybind11/eval.h>
#include <pybind11/embed.h>


static constexpr int numMPLandmarks = 33;

namespace RGBD {
	struct PoseInfo {
		PoseCoords* pc;
		double visibility;

		PoseInfo(PoseCoords* pc, double visibility) {
			this->pc = pc;
			this->visibility = visibility;
		}
	};
}

class Pose {
public:
#if USE_MEDIAPIPE_PYBIND
	/// <summary>
	/// Pose can be use used to get 3D coordinates from a person using a RGB-D sensor.
	/// </summary>
	/// <param name="cameraIntrinsics">Camera intrinsics parameters</param>
	/// <param name="imgWidth">The width of the image</param>
	/// <param name="imgHeight">The height of the image</param>
	/// <param name="mp">Mediapipe module. If Pose is used more than once, this must be used, since MediaPipe does not get unloaded correctly with CPython. 
	/// If you are using this, it's assumed you used py::initialize_interpreter() and will call py::finalize_interpreter() at the end.</param>
	Pose(RGBD::CameraIntrinsics cameraIntrinsics, int imgWidth, int imgHeigth, pybind11::module_ mp = pybind11::module_());
#else
	Pose(RGBD::CameraIntrinsics cameraIntrinsics, int imgWidth, int imgHeigth, std::string posePath = "");
#endif // USE_MEDIAPIPE_PYBIND
	~Pose();

	std::optional<std::array<RGBD::PoseInfo*, numMPLandmarks>> update(RGBD::RGB888Pixel* rgbImage, RGBD::DepthPixel* depthImage);

	std::optional<std::pair<std::array<RGBD::PoseInfo*, numMPLandmarks>, std::array<RGBD::PoseInfo*, numMPLandmarks>>> RGBDandMediaPipeUpdate(RGBD::RGB888Pixel* rgbImage, RGBD::DepthPixel* depthImage);

	std::optional<std::pair<std::array<RGBD::PoseInfo*, numMPLandmarks>, std::array<RGBD::PoseInfo*, numMPLandmarks>>> update2Dand3D(RGBD::RGB888Pixel* rgbImage, RGBD::DepthPixel* depthImage);

private:
	const int knn = 5;
	const int m_width;
	const int m_height;
	const int m_imgSize;

	PixelTo3D* pixelTo3D;
	std::string posePath;

	// mediapipe pybind bind variables
#if USE_MEDIAPIPE_PYBIND
	std::optional<std::pair<std::array<RGBD::PoseInfo*, numMPLandmarks>, std::optional<std::array<RGBD::PoseInfo*, numMPLandmarks>>>> shareImgGetLandmarks(RGBD::RGB888Pixel* rgbImage, bool getMP3D);

	pybind11::module_ mp;
	pybind11::object pose;
	bool pybindWasInitialized = false;
#else
	static std::string checkForPosePath();
	std::array<RGBD::PoseInfo*, numMPLandmarks> getCoords(std::string landmarks);

#ifdef USE_REDIS
	// redis
	redisContext* connection;
	void shareOnMemory(char* rgbImage);
	std::array<RGBD::PoseCoords*, numMPLandmarks> getFromMemory();
	std::array<RGBD::PoseCoords*, numMPLandmarks> get3DFromMemory();
	void setupRedis();
	bool checkIfLandmarks();
	void resetRedisLandmarkData();

	// return mp pose and pose world landmarks
	std::optional<std::pair<std::array<RGBD::PoseCoords*, numMPLandmarks>, std::array<RGBD::PoseCoords*, numMPLandmarks>>> shareAndGetFromMemory(RGBD::RGB888Pixel* rgbImage);
#else
	RGBD::Server* socketServer;
	std::optional<std::pair<std::array<RGBD::PoseInfo*, numMPLandmarks>, std::optional<std::array<RGBD::PoseInfo*, numMPLandmarks>>>> shareImgGetLandmarks(RGBD::RGB888Pixel* rgbImage, bool getMP3D);
#endif // USE_REDIS
#endif // USE_MEDIAPIPE_PYBIND
};