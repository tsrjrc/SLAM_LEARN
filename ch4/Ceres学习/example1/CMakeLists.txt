cmake_minimum_required(VERSION 2.8)
project(ceres)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable(use_ceres main.cpp)
target_link_libraries(use_ceres ${CERES_LIBRARIES})
