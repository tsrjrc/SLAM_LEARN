cmake_minimum_required( VERSION 2.8 )

project ( disparity )

set( CMAKE_BUILD_TYPE Debug )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# eigen 
include_directories( "/usr/include/eigen3/" )

# Pangolin 
find_package( Pangolin REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable( ${PROJECT_NAME} ${PROJECT_NAME}.cpp )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${Pangolin_LIBRARIES} )

