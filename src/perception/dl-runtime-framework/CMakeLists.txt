# This module cannot run independently
# It is meant to be a headerfile api ONLY.

cmake_minimum_required(VERSION 3.16)
project(dlRF CUDA CXX C)

set(SOURCES
    src/utils.cpp
    src/dlRF.cpp
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

message("asdadad: ${TORCH_INCLUDE_DIRS}")

if(CORE_DEBUG)
    # find_package(OpenCV REQUIRED)
    # message("hihihihihih: ${OpenCV_INCLUDE_DIRS}")
    add_library(${PROJECT_NAME} ${SOURCES})
    target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        ${MODULE_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        # ${CUDA_INCLUDE_DIRS}
        ${CUDAToolkit_INCLUDE_DIRS}
        ${TensorRT_INCLUDE_DIRS}
        ${TensorRT_DIR}/samples/common
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${TORCH_INCLUDE_DIRS}
    )
    target_link_directories(
        ${PROJECT_NAME}
        PUBLIC
        ${CUDAToolkit_LIBRARY_DIR}
    )
    target_link_libraries(
        ${PROJECT_NAME}
        PRIVATE
        ${OpenCV_LIBS}
        # ${CUDA_LIBRARIES} 
        ${TensorRT_LIBRARIES}
        ${LIBRARIES}
        ${DEPENDENCY_LIBRARIES}
        ${TORCH_LIBRARIES}
    )
    set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
    set(DEPENDENCY_LIBRARIES ${DEPENDENCY_LIBRARIES} ${PROJECT_NAME} PARENT_SCOPE)
else()
    set(CMAKE_MODULE_PATH "${ROOTDIR}/cmake" ${CMAKE_MODULE_PATH})
    find_package(MinuszeroPerceptionROSTemplate REQUIRED)

    add_library(${PROJECT_NAME} ${SOURCES})

    target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        ${MODULE_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${CUDAToolkit_INCLUDE_DIRS}
        ${TensorRT_INCLUDE_DIRS}
        ${xtl_INCLUDE_DIRS}
        ${xtensor_INCLUDE_DIRS}
        ${TensorRT_DIR}/samples/common
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${TORCH_INCLUDE_DIRS}
    )
    target_link_directories(
        ${PROJECT_NAME}
        PUBLIC
        ${CUDAToolkit_LIBRARY_DIR}
    )
    target_link_libraries(
        ${PROJECT_NAME}
        PRIVATE
        ${OpenCV_LIBS} 
        ${TensorRT_LIBRARIES}
        xtl
        xtensor
        Boost::serialization
        Boost::iostreams
        Boost::boost
        ${LIBRARIES}
        ${DEPENDENCY_LIBRARIES}
        ${TORCH_LIBRARIES}
    )
    target_compile_options(
        ${PROJECT_NAME}
        PRIVATE
        $<$<COMPILE_LANGUAGE:CUDA>:
        --use_fast_math
        -dopt=on
        -O3
        >
    )
    set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
    set(INCLUDE_INSTALL_PATH ${INCLUDE_INSTALL_DIR}/${PROJECT_NAME})
    set(MODULE_INCLUDE_DIRS ${MODULE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/install PARENT_SCOPE)
    set(MODULE_INSTALL_NAMES ${MODULE_INSTALL_NAMES} ${PROJECT_NAME} PARENT_SCOPE)
    set(MODULE_INSTALL_INCLUDE_DIRS ${MODULE_INSTALL_INCLUDE_DIRS} ${INCLUDE_INSTALL_PATH} PARENT_SCOPE)
    set(MODULE_INSTALL_LIBRARIES ${MODULE_INSTALL_LIBRARIES} ${LIB_INSTALL_PATH} PARENT_SCOPE)
endif()