cmake_minimum_required(VERSION 3.16)
project(perception_camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Ofast -DNDEBUG")

include(GNUInstallDirs)

find_package(OpenCV REQUIRED )
find_package(eCAL REQUIRED)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

set(CMAKE_MODULE_PATH "${ROOTDIR}/cmake" ${CMAKE_MODULE_PATH})

message(${CMAKE_MODULE_PATH})

set(SOURCES
    src/camera.cpp
    src/camera-stack.cpp
    run.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})
target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
  ${PROJECT_NAME}
    eCAL::core
    ${OpenCV_LIBRARIES}
    Threads::Threads
)
