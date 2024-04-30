cmake_minimum_required (VERSION 3.14)

project(RGBD-PixelTo3DWrappers VERSION "${CMAKE_PROJECT_VERSION}")

set(DEPENDENCIES RGBD-PixelTo3D)

if (BUILD_PYTHON_BINDINGS)
	message("Building Python Bindings")
	add_subdirectory(PyRGBDPixelTo3D)
endif()