#include "Python.hpp"

void def_pose_info(py::module& m) {
    py::class_<RGBD::PoseInfo> poseInfo(m, "PoseInfo", "Object for storing 3D coords (PoseCoords) and visibility");

    poseInfo.def(py::init<RGBD::PoseCoords*, double>(),
        "pc"_a, "visibility"_a);

    poseInfo.def_readwrite("pc", &RGBD::PoseInfo::pc);
    poseInfo.def_readwrite("visibility", &RGBD::PoseInfo::visibility);
}