cmake_minimum_required (VERSION 3.8)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project ("RGBD-PixelTo3D-Tests" DESCRIPTION "Tests for project")

Include(FetchContent)

set(DEPENDENCIES RGBD-PixelTo3D)

# catch
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG         v3.0.0-preview4
)

FetchContent_MakeAvailable(Catch2)

# opencv
FIND_PACKAGE( OpenCV REQUIRED )

# executable
add_executable(tests "TestsMatches.hpp" "ConvertTo3DTest.cpp" "PoseTest.cpp")

# set needed paths
target_compile_definitions(RGBD-PixelTo3D PUBLIC POSE_DIR="../..")
target_compile_definitions(tests PUBLIC TESTS_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# links
target_link_libraries(tests PRIVATE Catch2::Catch2 ${DEPENDENCIES})
target_link_libraries(tests PRIVATE ${OpenCV_LIBS})