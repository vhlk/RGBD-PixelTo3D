#include "Python.hpp"

void def_pixel_to_3d(py::module& m) {
    py::class_<PixelTo3D> pixelTo3d(m, "PixelTo3D", "3D coordinates from RGBD sensor.");
    
    pixelTo3d.def(py::init<RGBD::CameraIntrinsics>(),
            "cameraIntrinsics"_a);

    pixelTo3d.def("convertTo3D", py::overload_cast<int, int, double>(&PixelTo3D::convertTo3D), "Converts a pixel to 3D using camera intrinsics and depth", "pixelX"_a, "pixelY"_a, "depth"_a);

    pixelTo3d.def("convertTo3D", ([](PixelTo3D& self, py::buffer depthImage, int pixelX, int pixelY, int h, int w, int knn = 0) {
        py::buffer_info infoDepth = depthImage.request();

        if (infoDepth.ndim != 2) {
            throw std::runtime_error("Incompatible depth image dimension! (should have 2 dim, but has " + std::to_string(infoDepth.ndim) + ')');
        }

        return self.convertTo3D((RGBD::DepthPixel*)infoDepth.ptr, pixelX, pixelY, h, w, knn);
        }),
        "Converts a pixel to 3D using its depth and rgb image.\nkNN algoritm can be used to calculate the best (closer) depth for the ROI, ignoring 0 as it means it could not be estimated. Disabled by default.",
        "depthImage"_a, "pixelX"_a, "pixelY"_a, "imgHeight"_a, "imgWidth"_a, "knn"_a = 0);
}