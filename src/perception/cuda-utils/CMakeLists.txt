cmake_minimum_required(VERSION 3.16)
project(cuda-utils CUDA CXX C)

#-------------------------------------------------------------------------------
# Project files
#-------------------------------------------------------------------------------
set(SOURCES
    src/utils.cpp
    # src/blendImage.cu
    src/decodeLabel.cu
    # src/generateImage.cu
    # src/bgr_rgb.cu
    # src/imageBatchNormalization.cu
    # src/nhwc_nchw.cu
    # src/nonMaxSuppression.cu
    # src/detectYoloV5Objects.cu
    src/perception_kernels.cpp
)

if(CORE_DEBUG)
    add_library(${PROJECT_NAME} ${SOURCES})
    target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        ${MODULE_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${CUDAToolkit_INCLUDE_DIRS}
        ${TensorRT_INCLUDE_DIRS}
        ${TensorRT_DIR}/samples/common
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_directories(
        ${PROJECT_NAME}
        PUBLIC
        ${CUDAToolkit_LIBRARY_DIR}
    )
    target_link_libraries(
        ${PROJECT_NAME}
        PUBLIC
        ${OpenCV_LIBS} 
        ${TensorRT_LIBRARIES}
        ${LIBRARIES}
        ${DEPENDENCY_LIBRARIES}
    )
    set(DEPENDENCY_LIBRARIES ${DEPENDENCY_LIBRARIES} ${PROJECT_NAME} PARENT_SCOPE)
else()
    set(CMAKE_MODULE_PATH "${ROOTDIR}/cmake" ${CMAKE_MODULE_PATH})

    find_package(MinuszeroPerceptionROSTemplate REQUIRED)

    add_library(${PROJECT_NAME} ${SOURCES})

    message("Project Source Directroy: ${CMAKE_SOURCE_DIR}")
    target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        ${OpenCV_INCLUDE_DIRS}
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
        PUBLIC
        ${OpenCV_LIBS} 
        ${TensorRT_LIBRARIES}
        ${LIBRARIES}
        ${TORCH_LIBRARIES}
    )
    target_compile_options(
        ${PROJECT_NAME}
        PRIVATE
        $<$<COMPILE_LANGUAGE:CUDA>:
        --use_fast_math
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



