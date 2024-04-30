#include "RGBD-Pose.hpp"
#include <iostream>
#ifndef USE_MEDIAPIPE_PYBIND
#include <thread>
#endif // USE_MEDIAPIPE_PYBIND
#include <cmath>
#include <chrono>
#include <thread>
#include <filesystem>
#include <pybind11/numpy.h>

namespace py = pybind11;
using RGBD::CameraIntrinsics;
using RGBD::PoseCoords;
using RGBD::PoseInfo;

#if USE_REDIS
#define pose_wait() std::this_thread::sleep_for(std::chrono::milliseconds(10));

const char* setImage = "SET image %b";
const char* setWidth = "SET width %d";
const char* setHeigth = "SET height %d";
const char* getLandmarks = "GET landmarks";
const char* setLandmarks = "SET landmarks %s";
const char* getLandmarks3D = "GET landmarks3d";
const char* setLandmarks3D = "SET landmarks3d %s";
const char* getNoLandmarks = "GET noLandmarks";
const char* setNoLandmarks = "SET noLandmarks %s";
#else

#endif


Pose::Pose(CameraIntrinsics cameraIntrinsics, int imgWidth, int imgHeigth, pybind11::module_ mp) :
	m_width(imgWidth), m_height(imgHeigth), m_imgSize(m_width * m_height * 3) {
	pixelTo3D = new PixelTo3D(cameraIntrinsics);

	if (!mp.check()) {
		py::initialize_interpreter();
		mp = py::module_::import("mediapipe");
		pybindWasInitialized = true;
	} 
	py::detail::str_attr_accessor mp_pose = mp.attr("solutions").attr("pose");
	pose = mp_pose.attr("Pose")();
#if !USE_MEDIAPIPE_PYBIND
	// run mediapipe pose -> MediaPipe in windows is experimental
	// so we will be using the MediaPipe python wheel
	auto m_pose = [](std::string posePath) {
		int poseRes = -999;
		std::string _posePath = POSE_DIR;
		if (_posePath == "-1") {
			if (posePath == "")
				posePath = Pose::checkForPosePath();
		}
		else
			posePath = _posePath + std::string("/pose.py");

		std::string posePath1 = "python3 " + posePath;
		std::string posePath2 = "python " + posePath;
		
#ifndef NDEBUG
		std::cout << "Pose dir: " << posePath << "\n";
#endif // NDEBUG
#ifdef __unix__
#ifdef NDEBUG // don't show python messages in release mode
		posePath += " > /dev/null";
		posePath1 += " > /dev/null";
		posePath2 += " > /dev/null";
#endif // NDEBUG
		poseRes = system(posePath1.c_str());
		if (poseRes == 0) return;
		poseRes = system(posePath2.c_str());
		if (poseRes == 0) return;
#elif WIN32 || _WIN32 || __WIN32 && !__CYGWIN__
#ifdef NDEBUG // don't show python messages in release mode
		posePath += " >nul 2>nul";
		posePath1 += " >nul 2>nul";
		posePath2 += " >nul 2>nul";
#endif // NDEBUG
		if (posePath == "pose.py")
			poseRes = system(posePath.c_str());
		if (poseRes == 0) return;

		poseRes = system(posePath2.c_str());
		if (poseRes == 0) return;
		poseRes = system(posePath1.c_str());
		if (poseRes == 0) return;
#elif __CYGWIN__
#ifdef NDEBUG // don't show python messages in release mode
		posePath += " > /dev/null";
		posePath1 += " > /dev/null";
		posePath2 += " > /dev/null";
#endif // NDEBUG
		poseRes = system(posePath1.c_str());
		if (poseRes == 0) return;
		poseRes = system(posePath2.c_str());
		if (poseRes == 0) return;
#endif
#ifndef NDEBUG
		std::cout << "system result: " << poseRes << '\n';
#endif // NDEBUG
	};

	std::thread pose_thread(m_pose, posePath);

#if USE_REDIS
	setupRedis();
	redisReply* reply = (redisReply*)redisCommand(connection, setWidth, m_width);
	freeReplyObject(reply);
	reply = (redisReply*)redisCommand(connection, setHeigth, m_height);
	freeReplyObject(reply);

	resetRedisLandmarkData();
#else
	socketServer = new RGBD::Server();
	if (!socketServer->send(m_width))
		std::cout << "Could not send width\n";
	if (!socketServer->send(m_height))
		std::cout << "Could not send o height\n";
#endif

	pose_thread.detach();
	py::gil_scoped_release release;
#endif // USE_MEDIAPIPE_PYBIND

}

Pose::~Pose() {
	delete pixelTo3D;
#if !USE_MEDIAPIPE_PYBIND
	delete socketServer;
#endif
	if (pybindWasInitialized)
		py::finalize_interpreter();
}

std::optional<std::array<RGBD::PoseInfo*, numMPLandmarks>> Pose::update(RGBD::RGB888Pixel* rgbImage, RGBD::DepthPixel* depthImage) {
	auto rgbdMPUpd = RGBDandMediaPipeUpdate(rgbImage, depthImage);
	
	if (rgbdMPUpd.has_value()) [[likely]]
		return rgbdMPUpd.value().first;

	return std::nullopt;
}

std::optional<std::pair<std::array<RGBD::PoseInfo*, numMPLandmarks>, std::array<RGBD::PoseInfo*, numMPLandmarks>>> Pose::RGBDandMediaPipeUpdate(RGBD::RGB888Pixel* rgbImage, RGBD::DepthPixel* depthImage) {
#if USE_REDIS
	auto coordsMP = shareAndGetFromMemory(rgbImage);
#else
	auto optCoordsMP = shareImgGetLandmarks(rgbImage, true);
#endif

	if (!optCoordsMP.has_value()) [[unlikely]]
		return std::nullopt;

	auto coordsMP = optCoordsMP.value();

	// get pose landmarks in 3D using Pixel To 3D
	auto coordsMediaPipe2D = coordsMP.first;

	std::array<PoseInfo*, numMPLandmarks> coords3D;
	for (int i = 0; i < numMPLandmarks; i++) {
		auto mpCoord = coordsMediaPipe2D[i];
		auto pc3D = pixelTo3D->convertTo3D(depthImage, (int)(mpCoord->pc->x * m_width), (int)(mpCoord->pc->y * m_height), m_height, m_width, knn);
		coords3D[i] = new PoseInfo(pc3D, mpCoord->visibility);
	}

	return std::make_pair(coords3D, coordsMP.second.value());
}

std::optional<std::pair<std::array<RGBD::PoseInfo*, numMPLandmarks>, std::array<RGBD::PoseInfo*, numMPLandmarks>>> Pose::update2Dand3D(RGBD::RGB888Pixel* rgbImage, RGBD::DepthPixel* depthImage)
{
#if USE_REDIS
	auto coordsMP = shareAndGetFromMemory(rgbImage);
#else
	auto optCoordsMP = shareImgGetLandmarks(rgbImage, false);
#endif

	if (!optCoordsMP.has_value()) [[unlikely]]
		return std::nullopt;

	auto &coordsMP = optCoordsMP.value();

	// get pose landmarks in 3D using Pixel To 3D
	auto coordsMediaPipe2D = coordsMP.first;

	std::array<PoseInfo*, numMPLandmarks> coords3D;
	for (int i = 0; i < numMPLandmarks; i++) {
		auto mpCoord = coordsMediaPipe2D[i];
		auto pc3D = pixelTo3D->convertTo3D(depthImage, (int)(mpCoord->pc->x * m_width), (int)(mpCoord->pc->y * m_height), m_height, m_width, knn);
		coords3D[i] = new PoseInfo(pc3D, mpCoord->visibility);
	}

	return std::make_pair(coordsMediaPipe2D, coords3D);
}

#if !USE_MEDIAPIPE_PYBIND
std::array<RGBD::PoseInfo*, numMPLandmarks> Pose::getCoords(std::string landmarks) {
	std::array<RGBD::PoseInfo*, numMPLandmarks> pi;

	std::string temp = landmarks;

	const double divide = pow(10, 7);
	for (int i = 0; i < numMPLandmarks; i++) {
		int nextDelimiter = temp.find(";");
		std::string landmarkX = temp.substr(0, nextDelimiter);
		temp = temp.substr(nextDelimiter+1, temp.length());

		double x = std::stoi(landmarkX) / divide;

		nextDelimiter = temp.find(";");
		std::string landmarkY = temp.substr(0, nextDelimiter);
		temp = temp.substr(nextDelimiter+1, temp.length());

		double y = std::stoi(landmarkY) / divide;

		nextDelimiter = temp.find(";");
		std::string landmarkZ = temp.substr(0, nextDelimiter);
		temp = temp.substr(nextDelimiter+1, temp.length());

		double z = std::stoi(landmarkZ) / divide;

		nextDelimiter = temp.find(";");
		std::string landmarkVisibility = temp.substr(0, nextDelimiter);
		temp = temp.substr(nextDelimiter + 1, temp.length());

		double visibility = std::stoi(landmarkVisibility) / divide;

		PoseCoords* pc = new PoseCoords(x, y, z);
		pi[i] = new PoseInfo(pc, visibility);
	}

	return pi;
}
#endif // USE_MEDIAPIPE_PYBIND

#if USE_REDIS
void Pose::shareOnMemory(char* rgbImage) {
	redisReply* reply = (redisReply*)redisCommand(connection, setImage, rgbImage, m_height * m_width * 3);
	freeReplyObject(reply);
}

// check if a landmark was found
bool Pose::checkIfLandmarks() {
	redisReply* foundNoLandmarks = (redisReply*)redisCommand(connection, getNoLandmarks);
	while (foundNoLandmarks == NULL || foundNoLandmarks->len <= 0) {
		freeReplyObject(foundNoLandmarks);
		foundNoLandmarks = (redisReply*)redisCommand(connection, getNoLandmarks);
		pose_wait();
	}

	bool hasLandmarks = !(foundNoLandmarks->str == "true");

	freeReplyObject(foundNoLandmarks);

	return hasLandmarks;
}

// get landmarks from redis
std::array<PoseCoords*, numMPLandmarks> Pose::getFromMemory() {
	redisReply* reply = (redisReply*) redisCommand(connection, getLandmarks);
	while (reply == NULL || reply->len <= 0) {
		freeReplyObject(reply);
		reply = (redisReply*)redisCommand(connection, getLandmarks);
		pose_wait();
	}

	auto coords = getCoords(reply->str);

	freeReplyObject(reply);

	return coords;
}

// get 3dlandmarks from redis
std::array<PoseCoords*, numMPLandmarks> Pose::get3DFromMemory() {
	redisReply* reply = (redisReply*)redisCommand(connection, getLandmarks3D);
	while (reply == NULL || reply->len <= 0) {
		freeReplyObject(reply);
		reply = (redisReply*)redisCommand(connection, getLandmarks3D);
		pose_wait();
	}

	auto coords3D = getCoords(reply->str);

	freeReplyObject(reply);

	return coords3D;
}

std::optional<std::pair<std::array<RGBD::PoseCoords*, numMPLandmarks>, std::array<RGBD::PoseCoords*, numMPLandmarks>>> Pose::shareAndGetFromMemory(RGBD::RGB888Pixel* rgbImage)
{
	shareOnMemory((char*)rgbImage);

	if (!checkIfLandmarks()) return std::nullopt;

	auto coords2D = getFromMemory();
	auto coords3D = get3DFromMemory();

	// reset
	resetRedisLandmarkData();

	return std::make_pair(coords2D, coords3D);
}

void Pose::resetRedisLandmarkData() {
	redisReply* reply = (redisReply*)redisCommand(connection, setLandmarks, "");
	reply = (redisReply*)redisCommand(connection, setLandmarks3D, "");
	reply = (redisReply*)redisCommand(connection, setNoLandmarks, "");
	freeReplyObject(reply);
}

void Pose::setupRedis() {
	const char* hostname = "localhost";
	int port = 6379;

	//struct timeval timeout = { 2, 0 }; // 2 seconds
	connection = redisConnect(hostname, port);
	if (connection == NULL || connection->err) {
		if (connection)
			std::cerr << "Something bad happened: " << connection->errstr << std::endl;
		else
			std::cerr << "Something bad happened: redis is null" << std::endl;

		redisFree(connection);
		return;
	}
}
#else
std::optional<std::pair<std::array<RGBD::PoseInfo*, numMPLandmarks>, std::optional<std::array<RGBD::PoseInfo*, numMPLandmarks>>>> Pose::shareImgGetLandmarks(RGBD::RGB888Pixel* rgbImage, bool getMP3D)
{
#if !USE_MEDIAPIPE_PYBIND
	socketServer->send( (char*) rgbImage, m_imgSize);
	
	if (getMP3D)
		socketServer->send("Get MP 3D");
	else
		socketServer->send("No MP 3D");

	auto coords2D = socketServer->rcv();

	if (!coords2D.has_value()) [[unlikely]]
		return std::nullopt;

	if (getMP3D) {
		auto coords3D = socketServer->rcv();
		if (!coords3D.has_value()) [[unlikely]]
			return std::nullopt;
		
		auto convertedCoords2D = getCoords(coords2D.value());
		auto convertedCoords3D = getCoords(coords3D.value());
		return std::make_pair(convertedCoords2D, convertedCoords3D);
	}

	auto coords = getCoords(coords2D.value());
	return std::make_pair(coords, std::nullopt);
#else
	auto img = py::array(py::dtype::of<uint8_t>(), 
		std::vector{ m_height, m_width, 3 },
		rgbImage);
	py::object results = pose.attr("process")(img);
	auto pose_landmarks = results.attr("pose_landmarks");
	if (pose_landmarks.is_none()) [[unlikely]]
		return std::nullopt;
	auto landmarks = pose_landmarks.attr("landmark");

	std::array<RGBD::PoseInfo *, numMPLandmarks> landmarksList;
	auto it = landmarks.begin();
	int i = 0;
	while (it != py::iterator::sentinel()) {
		RGBD::PoseCoords* pc = new RGBD::PoseCoords((*it).attr("x").cast<double>(), (*it).attr("y").cast<double>(), (*it).attr("z").cast<double>());
		RGBD::PoseInfo* pi = new RGBD::PoseInfo(pc, (*it).attr("visibility").cast<double>());
		landmarksList[i] = pi;
		it++;
		i++;
	}

	auto landmarks3d = results.attr("pose_world_landmarks").attr("landmark");

	std::array<RGBD::PoseInfo*, numMPLandmarks> landmarks3dList;
	it = landmarks3d.begin();
	i = 0;
	while (it != py::iterator::sentinel()) {
		RGBD::PoseCoords* pc = new RGBD::PoseCoords((*it).attr("x").cast<double>(), (*it).attr("y").cast<double>(), (*it).attr("z").cast<double>());
		RGBD::PoseInfo* pi = new RGBD::PoseInfo(pc, (*it).attr("visibility").cast<double>());
		landmarks3dList[i] = pi;
		it++;
		i++;
	}

	return std::make_pair(landmarksList, landmarks3dList);
#endif // USE_MEDIAPIPE_PYBIND
}
#endif

#if !USE_MEDIAPIPE_PYBIND
std::filesystem::path findFilePath(std::filesystem::path currPath) {
	using namespace std::filesystem;

	for (const auto& file : directory_iterator(currPath)) {
		path p = path(file.path());
		if (file.is_regular_file() && p.filename().string() == "pose.py") {
			return p;
		}
		else if (file.is_directory()) {
			path recursion = findFilePath(p);
			if (recursion.string() != "")
				return recursion;
		}
	}

	return "";
}
#endif // USE_MEDIAPIPE_PYBIND

#if !USE_MEDIAPIPE_PYBIND
std::string Pose::checkForPosePath() {
	py::object scope = py::module_::import("__main__").attr("__dict__");
	if (py::eval("__name__", scope).cast<std::string>() == "__main__") { // we check if we are running from python code
		auto pathList = py::eval("exec('import PyRGBDPixelTo3D') or PyRGBDPixelTo3D.__path__", scope).cast<py::list>();
		return pathList[0].cast<std::string>() + "/pose.py";
	}

	using namespace std::filesystem;
	path currPath = current_path();	

	return findFilePath(currPath).string();
}
#endif //USE_MEDIAPIPE_PYBIND
