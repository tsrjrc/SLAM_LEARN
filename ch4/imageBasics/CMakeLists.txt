cmake_minimum_required( VERSION 2.8 )

project ( imageBasics )

set( CMAKE_CXX_FLAGS "-std=c++11" )

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( ${PROJECT_NAME} imageBasics.cpp )

target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )

