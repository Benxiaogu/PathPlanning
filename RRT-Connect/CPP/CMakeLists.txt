cmake_minimum_required(VERSION 3.5)
project(RRTConnect)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")


include_directories(include)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(rrtconnect src/main.cpp 
                    src/rrt_connect.cpp)
target_link_libraries(rrtconnect ${OpenCV_LIBS})