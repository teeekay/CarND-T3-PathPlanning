project(path-plan)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/WebSocketMessageHandler.cpp src/KeepLanePathPlanner.cpp src/KeepLanePathPlanner.h src/OutputPath.h src/PathPlannerInput.h src/CartesianPoint.h src/HighwayMap.cpp src/FrenetPoint.h src/PathPlanner.h src/SimpleSplineBasedPlanner.cpp src/SimpleSplineBasedPlanner.h src/spline.h src/OtherCar.h)


if(${CMAKE_SYSTEM_NAME} MATCHES "Linux") 
include_directories(/usr/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux") 

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows") 
#include_directories(C:\Users\tknight\AppData\Local\lxss\rootfs\usr\include)
#include_directories(%localappdata%\lxss\rootfs\usr\include)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows") 

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(path-plan ${sources})

target_link_libraries(path-plan z ssl uv uWS)
