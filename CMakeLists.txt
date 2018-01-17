project(SdcPathPlanning)

cmake_minimum_required (VERSION 3.5)

set(CMAKE_CXX_STANDARD 11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  include_directories(/usr/local/include)
  include_directories(/usr/local/opt/openssl/include)
  link_directories(/usr/local/lib)
  link_directories(/usr/local/opt/openssl/lib)
  link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

if(WIN32)
  message(USING WINDOWS)
  include_directories(../vcpkg/packages/uwebsockets_x86-windows/include)
  include_directories(../vcpkg/packages/openssl_x86-windows/include)
  include_directories(../vcpkg/packages/libuv_x86-windows/include)
  include_directories(../vcpkg/packages/zlib_x86-windows/include)
endif()

add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
