project(path-planning)

cmake_minimum_required (VERSION 3.5)

set(CMAKE_CXX_STANDARD 11)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

add_definitions(-D_USE_MATH_DEFINES -DSSL_MODE_RELEASE_BUFFERS)

# @todo Check whether we need these
# find_package(ZLIB)
# find_package(libuv)
# find_package(ssl)
# find_package(uWS)

set(headers 
  src/BehavioralPlanner.h
  src/CartesianPoint.h
  src/Cost.h
  src/DrivingStateMachine.h
  src/FrenetPoint.h
  src/FsmList.h
  src/HighwayMap.h
  src/KeepLanePathPlanner.h
  src/LaneChangePlanner.h
  src/OtherCar.h
  src/OutputPath.h  
  src/PathPlanner.h
  src/PathPlannerInput.h
  src/Playground.h
  src/Road.h
  src/SimpleSplineBasedPlanner.h
  src/Vehicle.h
  src/WebSocketMessageHandler.h
)

set(sources 
  src/BehavioralPlanner.cpp
  src/Cost.cpp
  src/DrivingStateMachine.cpp
  src/HighwayMap.cpp
  src/KeepLanePathPlanner.cpp
  src/LaneChangePlanner.cpp
  src/Playground.cpp
  src/Road.cpp
  src/SimpleSplineBasedPlanner.cpp
  src/Vehicle.cpp
  src/WebSocketMessageHandler.cpp
  src/pid/pid.cpp
)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 
  include_directories(/usr/local/include)
  include_directories(/usr/local/opt/openssl/include)
  link_directories(/usr/local/lib)
  link_directories(/usr/local/opt/openssl/lib)
  link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

if(WIN32)
  # @todo Check whether we need these
  # target_include_directories(path-planning PRIVATE src)
  # target_include_directories(path-planning PRIVATE src/libuv)
  # target_include_directories(path-planning PRIVATE src/zlib-1.2.11)
endif()

# Lib
add_library(path-planning-lib ${headers} ${sources})
target_include_directories(path-planning-lib PUBLIC src)
target_link_libraries(path-planning-lib  PUBLIC z ssl uv uWS)

# Main executable
add_executable(path-planning src/main.cpp)
target_link_libraries(path-planning path-planning-lib)

# Test executable
add_executable(path-planning-test test/main.cpp)
target_include_directories(path-planning-test  PRIVATE test)
target_link_libraries(path-planning-test  path-planning-lib)