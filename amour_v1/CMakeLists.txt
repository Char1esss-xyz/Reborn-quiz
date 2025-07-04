cmake_minimum_required( VERSION 3.14)

project( DisplayImage )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable(amour_v1 amour.cpp )
target_link_libraries( amour_v1 ${OpenCV_LIBS} )