#include "Python.hpp"

void def_camera_intrinsics(py::module &m) {
    py::class_<RGBD::CameraIntrinsics> camera_intrinsics(m, "CameraIntrinsics", "Object for storing camera intrinsics.");

    camera_intrinsics.def(py::init<double, double, double, double, double>(),
            "focalX"_a, "focalY"_a, "offsetX"_a, "offsetY"_a, "skew"_a = 0);

    camera_intrinsics.def_readwrite("focalX", &RGBD::CameraIntrinsics::focalX);
    camera_intrinsics.def_readwrite("focalY", &RGBD::CameraIntrinsics::focalY);
    camera_intrinsics.def_readwrite("offsetX", &RGBD::CameraIntrinsics::offsetX);
    camera_intrinsics.def_readwrite("offsetY", &RGBD::CameraIntrinsics::offsetY);
    camera_intrinsics.def_readwrite("skew", &RGBD::CameraIntrinsics::skew);
}