#include <RGBD-PixelTo3D.hpp>
#include <RGBD-Pose.hpp>
#include <AuxImageType.hpp>
#include "Python.hpp"

PYBIND11_MODULE(PyRGBDPixelTo3D, m) {
    m.doc() = "3D coordinates from RGBD sensor. 3D Human Pose using RGBD sensors.";
    m.attr("__version__") = PyRGBDPixelTo3D_VERSION;

    def_camera_intrinsics(m);
    def_pixel_to_3d(m);
    def_pose_3d(m);
    def_pose_coords(m);
    def_pose_info(m);
}