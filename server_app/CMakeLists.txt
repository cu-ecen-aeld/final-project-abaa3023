# Enable C++11
# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
cmake_minimum_required(VERSION 2.8)
project( server )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( server server.cpp )
target_link_libraries( server ${OpenCV_LIBS} )
