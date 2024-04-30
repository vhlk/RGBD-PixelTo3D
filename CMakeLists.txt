cmake_minimum_required (VERSION 3.14)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project ("RGBD-PixelTo3D" VERSION 0.9 DESCRIPTION "3D Pose for RGB-D cameras and RGBD 2D to 3D converter")

include(ExternalProject)
include(FetchContent)

# hiredis
if (USE_REDIS) # disabled by default
	include_directories(RGBD-PixelTo3D PUBLIC 3rdparty/hiredis)
	AUX_SOURCE_DIRECTORY(3rdparty/hiredis hiredis_files)
	list(REMOVE_ITEM hiredis_files "3rdparty/hiredis/ssl.c")
	list(REMOVE_ITEM hiredis_files "3rdparty/hiredis/test.c")
	if(WIN32)
		link_libraries(ws2_32 wsock32)
	endif(WIN32)
endif()

if (NOT USE_REDIS) # uses socket (asio) by default
# asio (from boost)
    find_package(Threads REQUIRED)

    # ASIO doesn't use CMake, we have to configure it manually. Extra notes for using on Windows:
    #
    # 1) If _WIN32_WINNT is not set, ASIO assumes _WIN32_WINNT=0x0501, i.e. Windows XP target, which is
    # definitely not the platform which most users target.
    #
    # 2) WIN32_LEAN_AND_MEAN is defined to make Winsock2 work.
    add_library(asio INTERFACE)

    target_include_directories(asio SYSTEM INTERFACE 3rdparty/asio)

    target_compile_definitions(asio INTERFACE ASIO_STANDALONE ASIO_NO_DEPRECATED)

    target_link_libraries(asio INTERFACE Threads::Threads)

    if(WIN32)
    # macro see @ https://stackoverflow.com/a/40217291/1746503
    macro(get_win32_winnt version)
        if(CMAKE_SYSTEM_VERSION)
        set(ver ${CMAKE_SYSTEM_VERSION})
        string(REGEX MATCH "^([0-9]+).([0-9])" ver ${ver})
        string(REGEX MATCH "^([0-9]+)" verMajor ${ver})
        # Check for Windows 10, b/c we'll need to convert to hex 'A'.
        if("${verMajor}" MATCHES "10")
            set(verMajor "A")
            string(REGEX REPLACE "^([0-9]+)" ${verMajor} ver ${ver})
        endif("${verMajor}" MATCHES "10")
        # Remove all remaining '.' characters.
        string(REPLACE "." "" ver ${ver})
        # Prepend each digit with a zero.
        string(REGEX REPLACE "([0-9A-Z])" "0\\1" ver ${ver})
        set(${version} "0x${ver}")
        endif()
    endmacro()

    if(NOT DEFINED _WIN32_WINNT)
        get_win32_winnt(ver)
        set(_WIN32_WINNT ${ver})
    endif()

    message(STATUS "Set _WIN32_WINNET=${_WIN32_WINNT}")

    target_compile_definitions(asio INTERFACE _WIN32_WINNT=${_WIN32_WINNT} WIN32_LEAN_AND_MEAN)
    endif()
endif()

# eigen
set(EIGEN_BUILD_DIR   ${CMAKE_BINARY_DIR}/eigen)
set(EIGEN_INSTALL_DIR ${CMAKE_SOURCE_DIR}/3rdparty/installs/eigen3)
set(EIGEN_INCLUDE_DIR ${EIGEN_INSTALL_DIR})

ExternalProject_Add(
    eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    PREFIX ${EIGEN_BUILD_DIR}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND
      ${CMAKE_COMMAND} -E copy_directory ${EIGEN_BUILD_DIR}/src/eigen/Eigen ${EIGEN_INCLUDE_DIR}/Eigen 
      && ${CMAKE_COMMAND} -E copy_directory ${EIGEN_BUILD_DIR}/src/eigen/unsupported ${EIGEN_INCLUDE_DIR}/unsupported
    TEST_COMMAND ""
)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIR})

# include files
include_directories(RGBD-PixelTo3D "include")

# pybind
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://www.github.com/pybind/pybind11
    GIT_TAG        v2.9.1
)

FetchContent_GetProperties(pybind11)
if(NOT pybind11_POPULATED)
    FetchContent_Populate(pybind11)
    add_subdirectory(${pybind11_SOURCE_DIR} ${pybind11_BINARY_DIR})
endif()

# set src files
set(SOURCE_FILES
    "src/main/RGBD-PixelTo3D.cpp"
    "src/main/RGBD-Pose.cpp"
)

# set header files
set(HEADER_FILES
    "include/RGBD-PixelTo3D.hpp" 
    "include/RGBD-Pose.hpp"
    "include/AuxImageType.hpp"
    ${hiredis_files}
)

# create library
add_library(RGBD-PixelTo3D
	 ${HEADER_FILES} ${SOURCE_FILES}  
)
# Adicione a origem ao executável deste projeto.
# add_executable (RGBD-PixelTo3D "src/main/RGBD-PixelTo3D.cpp" "include/RGBD-PixelTo3D.hpp" "src/main/RGBD-Pose.cpp" "include/RGBD-Pose.hpp" ${hiredis_files} "include/AuxImageType.hpp")

# set public header
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")

# set pose path
target_compile_definitions(RGBD-PixelTo3D PUBLIC POSE_DIR="-1")

# copy pose.py
# if (NOT USE_REDIS)
#     add_custom_command(
# 	    TARGET RGBD-PixelTo3D POST_BUILD
# 	    COMMAND ${CMAKE_COMMAND} -E copy
# 	    ${CMAKE_SOURCE_DIR}/pose.py $<TARGET_FILE_DIR:RGBD-PixelTo3D>
#         DEPENDS ${CMAKE_SOURCE_DIR}/pose.py
#     )
# else()
#     add_custom_command(
# 	    TARGET RGBD-PixelTo3D POST_BUILD
# 	    COMMAND ${CMAKE_COMMAND} -E copy
# 	    ${CMAKE_SOURCE_DIR}/redis-pose.py $<TARGET_FILE_DIR:RGBD-PixelTo3D>
#         DEPENDS ${CMAKE_SOURCE_DIR}/redis-pose.py
#     )
# endif()

# tests
set(TESTING_ENV "" CACHE BOOL "For testing environment. Not suitable for production.")
if (TESTING_ENV)
    message("COMPILING TESTS")
	add_subdirectory(src/test)
endif()

# links
if (NOT USE_REDIS)
    target_link_libraries(${PROJECT_NAME} asio)
endif()
add_dependencies(${PROJECT_NAME} eigen)

# install lib
install(TARGETS ${PROJECT_NAME}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin 
PUBLIC_HEADER DESTINATION include)

# wrappers
set(BUILD_PYTHON_BINDINGS "" CACHE BOOL "Build Python wrappers")
add_subdirectory(wrappers)

# pybind
target_link_libraries(${PROJECT_NAME} pybind11::module pybind11::lto pybind11::windows_extras)