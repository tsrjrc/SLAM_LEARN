cmake_minimum_required( VERSION 2.8 )
project( g2o_curve_fitting )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3")

# 添加cmake模块以使用g2o库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

#G2O
find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )

# eigen 
include_directories( "/usr/include/eigen3/" )

find_package(Boost COMPONENTS system iostreams filesystem REQUIRED)

add_executable( ${PROJECT_NAME} ${PROJECT_NAME}.cpp )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} g2o_core g2o_stuff ${Boost_LIBRARIES})
