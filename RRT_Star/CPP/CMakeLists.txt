cmake_minimum_required(VERSION 3.5)
project(RRTStar)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(rrtstar src/main.cpp 
                    src/rrt_star.cpp)
target_link_libraries(rrtstar ${OpenCV_LIBS})