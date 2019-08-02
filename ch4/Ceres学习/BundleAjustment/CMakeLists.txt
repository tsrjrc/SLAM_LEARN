cmake_minimum_required( VERSION 2.8 )
project( simple_bundle_adjuster )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3")

# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
find_package(Boost COMPONENTS system iostreams filesystem REQUIRED)

# Ceres 
find_package( Ceres REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS} )


add_executable( ${PROJECT_NAME} ${PROJECT_NAME}.cc )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${Boost_LIBRARIES})
