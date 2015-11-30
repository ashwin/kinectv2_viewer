project(kinectv2_viewer)
cmake_minimum_required(VERSION 2.8)

set(
    CMAKE_RUNTIME_OUTPUT_DIRECTORY
    ${CMAKE_HOME_DIRECTORY}
    )

include_directories(
    "C:/Program Files/opencv/build/include"
    "$ENV{KINECTSDK20_DIR}/inc"
    )
		
link_directories(
    "C:/Program Files/opencv/build/x64/vc12/lib"
    "$ENV{KINECTSDK20_DIR}/Lib/x64"
    )
		
link_libraries(
    Kinect20.lib
    optimized opencv_world300.lib
    debug opencv_world300d.lib
    )

add_executable(
    kinectv2_viewer
    src/main.cpp
    )
