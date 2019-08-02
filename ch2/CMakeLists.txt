cmake_minimum_required( VERSION 2.8 )

project( HelloSLAM )


set( CMAKE_BUILD_TYPE "Release" )

add_library( hello SHARED hello.cpp )

add_executable( sayhello useHello.cpp )

target_link_libraries( sayhello hello)

INSTALL( TARGETS hello LIBRARY DESTINATION /usr/local/lib )
INSTALL( FILES ${PROJECT_SOURCE_DIR}/hello.h DESTINATION /usr/local/include )


