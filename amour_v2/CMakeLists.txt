cmake_minimum_required( VERSION 3.14)

project( amour_v2 )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable(amour_v2 main.cpp )
target_link_libraries( amour_v2 ${OpenCV_LIBS} )