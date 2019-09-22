cmake_minimum_required(VERSION 2.8)

project(seam_carving)

find_package(OpenCV REQUIRED)
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})

add_subdirectory(seam_carver)
add_subdirectory(app)

