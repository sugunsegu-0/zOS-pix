
# This module cannot run independently
# It is meant to be a headerfile api ONLY.

cmake_minimum_required(VERSION 3.16)
project(commons CXX C)

include(GNUInstallDirs)

set(CMAKE_C_STANDARD 99)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Ofast -DNDEBUG")

# For finding FindTensorRT.cmake
set(CMAKE_MODULE_PATH "${ROOTDIR}/cmake" ${CMAKE_MODULE_PATH})

set(SOURCES
    commons.cpp
)

find_package(OpenCV REQUIRED)
find_package(Boost CONFIG REQUIRED)
find_package(datastructures REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(glfw3 REQUIRED)
find_package(GLEW REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )
link_libraries(GLEW::GLEW ${OPENGL_LIBRARIES} glfw ${GLUT_LIBRARY})

add_library(${PROJECT_NAME} SHARED ${SOURCES})
message("Project Source Directroy: ${CMAKE_SOURCE_DIR}")
target_include_directories(
    ${PROJECT_NAME} 
    PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/IPC
    ${CMAKE_CURRENT_SOURCE_DIR}/boost_serial
)

target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC 
    ${OpenCV_LIBS}
    Boost::boost
)