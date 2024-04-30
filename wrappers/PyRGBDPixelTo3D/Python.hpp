#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// common includes
#include <RGBD-Pose.hpp>
#include <RGBD-PixelTo3D.hpp>

namespace py = pybind11;
using namespace pybind11::literals;

// Partial function definitions
void def_pixel_to_3d(py::module& m);
void def_pose_3d(py::module& m);
void def_camera_intrinsics(py::module& m);
void def_pose_coords(py::module& m);
void def_pose_info(py::module& m);