cmake_minimum_required(VERSION 3.5)
project(DStar)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(dstar src/main.cpp 
                    src/dstar.cpp
                    src/map.cpp)
target_link_libraries(dstar ${OpenCV_LIBS})