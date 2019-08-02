CMake_minimum_required( VERSION 2.8 )
project( draw_trajectory2 )

set( CMAKE_CXX_FLAGS "-std=c++11" )


find_package( Eigen3 3.1.0 REQUIRED )
find_package( Pangolin REQUIRED )
find_package( OpenCV REQUIRED )
find_package( Sophus REQUIRED )

include_directories(
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${Sophus_INCLUDE_DIRS}
)


add_executable( draw_trajectory2 draw_trajectory2.cpp )

target_link_libraries(draw_trajectory2 
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${Sophus_LIBRARIES}
)
