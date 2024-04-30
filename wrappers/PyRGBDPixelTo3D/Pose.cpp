#include "Python.hpp"

void def_pose_3d(py::module& m) {
    py::class_<Pose> pose3d(m, "Pose", "3D Human Pose using RGBD sensors.");

    pose3d.def(py::init<RGBD::CameraIntrinsics, int, int, py::module_>(),
            "cameraIntrinsics"_a, "imgWidth"_a, "imgHeight"_a, "mediapipeModule"_a);

    pose3d.def("update", [](Pose& self, py::buffer rgbImage, py::buffer depthImage) {
        py::buffer_info infoRGB = rgbImage.request();
        py::buffer_info infoDepth = depthImage.request();

        if (infoRGB.ndim != 3) {
            throw std::runtime_error("Incompatible RGB image dimension! (should have 3 dims, but has " + std::to_string(infoRGB.ndim) + ')');
        }
        if (infoDepth.ndim != 2) {
            throw std::runtime_error("Incompatible depth image dimension! (should have 2 dim, but has " + std::to_string(infoDepth.ndim) + ')');
        }

        return self.update((RGBD::RGB888Pixel*)infoRGB.ptr, (RGBD::DepthPixel*)infoDepth.ptr);
        }, "rgbImage"_a, "depthImage"_a);
    pose3d.def("RGBDandMediaPipeUpdate", [](Pose& self, py::buffer rgbImage, py::buffer depthImage) {
        py::buffer_info infoRGB = rgbImage.request();
        py::buffer_info infoDepth = depthImage.request();

        if (infoRGB.ndim != 3) {
            throw std::runtime_error("Incompatible RGB image dimension! (should have 3 dims, but has " + std::to_string(infoRGB.ndim) + ')');
        }
        if (infoDepth.ndim != 2) {
            throw std::runtime_error("Incompatible depth image dimension! (should have 2 dim, but has " + std::to_string(infoDepth.ndim) + ')');
        }

        return self.RGBDandMediaPipeUpdate((RGBD::RGB888Pixel*)infoRGB.ptr, (RGBD::DepthPixel*)infoDepth.ptr);
    }, "rgbImage"_a, "depthImage"_a);
    pose3d.def("update2Dand3D", [](Pose& self, py::buffer rgbImage, py::buffer depthImage) {
        py::buffer_info infoRGB = rgbImage.request();
        py::buffer_info infoDepth = depthImage.request();

        if (infoRGB.ndim != 3) {
            throw std::runtime_error("Incompatible RGB image dimension! (should have 3 dims, but has " + std::to_string(infoRGB.ndim) + ')');
        }
        if (infoDepth.ndim != 2) {
            throw std::runtime_error("Incompatible depth image dimension! (should have 2 dim, but has " + std::to_string(infoDepth.ndim) + ')');
        }

        return self.update2Dand3D((RGBD::RGB888Pixel*)infoRGB.ptr, (RGBD::DepthPixel*)infoDepth.ptr);
        }, "rgbImage"_a, "depthImage"_a);
}