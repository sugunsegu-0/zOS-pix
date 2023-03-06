cmake_minimum_required(VERSION 3.16)
project(core-debug CUDA CXX)

set(CORE_DEBUG ON)

# Set C++ version and optimization level

#-------------------------------------------------------------------------------
# CUDA host compiler must be set before CUDA is enabled as a language
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
# Set CUDA_DIR
#-------------------------------------------------------------------------------
if (DEFINED CUDA_DIR)
    if((DEFINED CUDA_TOOLKIT_ROOT_DIR) AND (NOT CUDA_TOOLKIT_ROOT_DIR STREQUAL CUDA_DIR))
        message(FATAL_ERROR "Cannot set both CUDA_DIR and (legacy) CUDA_TOOLKIT_ROOT_DIR")
    endif()
elseif (DEFINED CUDA_TOOLKIT_ROOT_DIR)
    message(WARNING "Please set CUDA_DIR instead of (legacy) CUDA_TOOLKIT_ROOT_DIR")
    set(CUDA_DIR  ${CUDA_TOOLKIT_ROOT_DIR} CACHE PATH "CUDA Toolkit location.")
else()
    set(CUDA_DIR  "/usr/local/cuda/" CACHE PATH "CUDA Toolkit location.")
endif()
if(NOT CMAKE_CUDA_COMPILER)
    set(CMAKE_CUDA_COMPILER "${CUDA_DIR}/bin/nvcc")
endif()
set(CMAKE_CUDA_HOST_COMPILER ${CMAKE_CXX_COMPILER})
enable_language(CUDA)

set(CMAKE_CUDA_STANDARD 11)
set(CMAKE_C_STANDARD 99)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Ofast -DNDEBUG")

# For finding FindTensorRT.cmake
set(CMAKE_MODULE_PATH "./cmake" ${CMAKE_MODULE_PATH})

# Use the correct version of CUDA
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)
set(CUDA_LINK_LIBRARIES_KEYWORD PUBLIC)
set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
find_package(CUDAToolkit REQUIRED)
message(STATUS "Found CUDA ${CUDA_VERSION_STRING} at ${CUDA_TOOLKIT_ROOT_DIR}")
find_package(OpenCV CONFIG REQUIRED)
find_package(TensorRT REQUIRED)
find_package(xtensor CONFIG REQUIRED)
find_package(Torch CONFIG REQUIRED)

#################################################################################
#                   BUILDING PERCEPTION DEPENDENCIES                            #
#################################################################################

set(DEPENDENCY_LIBRARIES)

set(DEPENDENCY_DIRECTORIES
    ${CMAKE_SOURCE_DIR}/../cuda-utils
    ${CMAKE_SOURCE_DIR}/../dl-runtime-framework
)
set(DEPENDENCY_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

foreach(dependency_icl_dir IN LISTS DEPENDENCY_DIRECTORIES)
    list(APPEND MODULE_INCLUDE_DIRS ${dependency_icl_dir}/include)
endforeach()

foreach(dependency IN LISTS DEPENDENCY_DIRECTORIES)
    file(RELATIVE_PATH
        DEPENDENCY_FOLDER # Output variable
        ${CMAKE_SOURCE_DIR}/../ # Base directory
        ${dependency} # Absolute path to the file
    )
    add_subdirectory(${dependency} ${CMAKE_SOURCE_DIR}/build/${DEPENDENCY_FOLDER}/build)
endforeach()

#################################################################################
#                   BUILDING PERCEPTION MODULE                                  #
#################################################################################

if(RENDER_PERCEPTION)
    add_definitions(-DRENDER_PERCEPTION)
endif()

set(MODULE_DIRECTORIES
    ${CMAKE_SOURCE_DIR}/../src/object-detection
    ${CMAKE_SOURCE_DIR}/../src/segmentation
    ${CMAKE_SOURCE_DIR}/../src/lane-estimation
    ${CMAKE_SOURCE_DIR}/../src/freespace
)

foreach(module_icl_dir IN LISTS MODULE_DIRECTORIES)
    list(APPEND MODULE_INCLUDE_DIRS ${module_icl_dir}/include)
endforeach()

foreach(module IN LISTS MODULE_DIRECTORIES)
    file(RELATIVE_PATH
        MODULE_FOLDER # Output variable
        ${CMAKE_SOURCE_DIR}/../src # Base directory
        ${module} # Absolute path to the file
    )
    add_subdirectory(${module} ${CMAKE_SOURCE_DIR}/build/${MODULE_FOLDER}/build)
endforeach()

##################################################################################
#                   BUILDING PERCEPTION EXECUTABLE                               #
##################################################################################
set(SOURCES
    src/perception.cpp
    core-debug.cpp
)
set(PERCEPTION_EXECUTABLE_NAME
    ${PROJECT_NAME}.run
)
add_executable(${PERCEPTION_EXECUTABLE_NAME} ${SOURCES})
target_include_directories(
    ${PERCEPTION_EXECUTABLE_NAME}
    PUBLIC
    ${OpenCV_INCLUDE_DIRS} 
    ${CUDAToolkit_INCLUDE_DIRS}
    ${TensorRT_INCLUDE_DIRS} 
    ${xtensor_INCLUDE_DIRS}
    ${TensorRT_DIR}/samples/common
    ${DEPENDENCY_INCLUDE_DIRS}
    ${MODULE_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)     
target_link_directories(
    ${PERCEPTION_EXECUTABLE_NAME}
    PUBLIC
    ${CUDAToolkit_LIBRARY_DIR}
)
target_link_libraries(
    ${PERCEPTION_EXECUTABLE_NAME}
    PUBLIC 
    ${OpenCV_LIBS}
    ${TensorRT_LIBRARIES} 
    xtensor
    ${DEPENDENCY_LIBRARIES}
    ${LIBRARIES}
)