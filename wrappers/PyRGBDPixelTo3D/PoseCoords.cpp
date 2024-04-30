#include "Python.hpp"

void def_pose_coords(py::module& m) {
    py::class_<RGBD::PoseCoords> poseCoords(m, "PoseCoords", "Object for storing 3D coords");

    poseCoords.def(py::init<double, double, double>(),
        "x"_a, "y"_a, "z"_a);

    poseCoords.def_readwrite("x", &RGBD::PoseCoords::x);
    poseCoords.def_readwrite("y", &RGBD::PoseCoords::y);
    poseCoords.def_readwrite("z", &RGBD::PoseCoords::z);
}