cmake_minimum_required (VERSION 3.14)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project (PyRGBDPixelTo3D VERSION "${CMAKE_PROJECT_VERSION}" DESCRIPTION "RGBD-PixelTo3D PythonWrapper")

include(FetchContent)

set(DEPENDENCIES RGBD-PixelTo3D)

# srcs
set(SOURCE_FILES
    "Python.cpp"
    "Pose.cpp"
    "CameraIntrinsics.cpp"
    "PixelTo3D.cpp"
    "PoseCoords.cpp"
    "PoseInfo.cpp"
)

# headers
set(HEADER_FILES
    "Python.hpp"
)

pybind11_add_module(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})

# pass current lib version
add_compile_definitions(PyRGBDPixelTo3D_VERSION="${CMAKE_PROJECT_VERSION}")

# copy pose.py to here
# add_custom_command(
# 	    TARGET ${PROJECT_NAME} PRE_BUILD
# 	    COMMAND ${CMAKE_COMMAND} -E copy
# 	    ${CMAKE_CURRENT_SOURCE_DIR}/../../pose.py ${CMAKE_CURRENT_SOURCE_DIR}
# )

target_link_libraries(${PROJECT_NAME} PRIVATE ${DEPENDENCIES})

install(TARGETS ${PROJECT_NAME}
  DESTINATION .
)