project(path-planning)

cmake_minimum_required (VERSION 3.5)

set(CMAKE_CXX_STANDARD 11)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

add_definitions(-D_USE_MATH_DEFINES -DSSL_MODE_RELEASE_BUFFERS)

set(sources 
  src/KeepLanePathPlanner.h
  src/KeepLanePathPlanner.cpp
  src/OutputPath.h
  src/PathPlannerInput.h
  src/CartesianPoint.h
  src/HighwayMap.h
  src/HighwayMap.cpp
  src/FrenetPoint.h
  src/PathPlanner.h
  src/SimpleSplineBasedPlanner.h
  src/SimpleSplineBasedPlanner.cpp
  src/spline.h
  src/OtherCar.h
  src/WebSocketMessageHandler.h
  src/WebSocketMessageHandler.cpp
  src/main.cpp
)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 
  include_directories(/usr/local/include)
  include_directories(/usr/local/opt/openssl/include)
  link_directories(/usr/local/lib)
  link_directories(/usr/local/opt/openssl/lib)
  link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

if(WIN32)
  # @todo
endif()

add_executable(path-planning ${sources})

target_link_libraries(path-planning z ssl uv uWS)

target_include_directories(path-planning PRIVATE src)
target_include_directories(path-planning PRIVATE src/libuv)
target_include_directories(path-planning PRIVATE src/zlib-1.2.11)
