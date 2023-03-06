# This module cannot run independently
# It is meant to be a headerfile api ONLY.

cmake_minimum_required(VERSION 3.16)
project(freespace CUDA CXX C)

if(RENDER_PERCEPTION_FREESPACE)
    find_package(OpenGL REQUIRED)
    find_package(GLUT REQUIRED)
    find_package(glfw3 REQUIRED)
    find_package(GLEW REQUIRED)

    include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )
    link_libraries( GLEW::GLEW ${OPENGL_LIBRARIES} glfw ${GLUT_LIBRARY} )
    add_definitions(-DRENDER_PERCEPTION_FREESPACE)
endif()

if(CORE_DEBUG)
    set(SOURCES
        src/freespace.cpp
    )
    add_library(${PROJECT_NAME} ${SOURCES})
    target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        ${MODULE_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${TensorRT_INCLUDE_DIRS}
        ${TensorRT_DIR}/samples/common
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(
        ${PROJECT_NAME}
        PUBLIC
        ${CUDA_LIBRARIES} 
        ${OpenCV_LIBS} 
        ${TensorRT_LIBRARIES}
        ${LIBRARIES}
        ${DEPENDENCY_LIBRARIES}
    )
    set(DEPENDENCY_LIBRARIES ${DEPENDENCY_LIBRARIES} ${PROJECT_NAME} PARENT_SCOPE)

elseif(CORE_DEVELOPMENT)
    set(CMAKE_MODULE_PATH "${ROOTDIR}/cmake" ${CMAKE_MODULE_PATH})
    find_package(MinuszeroPerceptionROSTemplate REQUIRED)
    set(SOURCES
        src/freespace.cpp
        # ros/freespace_node_ros.cpp
        # ros/run_freespace_ros.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../node.cpp
    )
    add_executable(${PROJECT_NAME} ${SOURCES})
    message("Project Source Directroy: ${CMAKE_CURRENT_SOURCE_DIR}")
    target_include_directories(
        ${PROJECT_NAME} 
        PUBLIC
        ${OpenCV_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${TensorRT_INCLUDE_DIRS}
        ${TensorRT_DIR}/samples/common
        ${MODULE_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/ros
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
        ${PERCEPTION_DATA_STRUCTURES_INCLUDE_DIR}

        # ros include dirs
        ${Boost_INCLUDE_DIRS}
        # ${rclcpp_INCLUDE_DIRS}
        ${sensor_INCLUDE_DIRS}
        ${minuszero_msgs_INCLUDE_DIRS}
        ${cv_bridge_INCLUDE_DIRS}
        ${message_filters_INCLUDE_DIRS}
        ${image_transport_INCLUDE_DIRS}
    )
    target_link_libraries(
        ${PROJECT_NAME}
        PUBLIC 
        ${CUDA_LIBRARIES} 
        ${OpenCV_LIBS} 
        ${TensorRT_LIBRARIES}
        ${LIBRARIES}
        # datastructures::perception

        # ros libraries
        ${MODULE_INSTALL_NAMES}
        ${rclcpp_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        ${minuszero_msgs_LIBRARIES}
        ${cv_bridge_LIBRARIES}
        ${message_filters_LIBRARIES}
    )
    set(INSTALL_NAMES ${INSTALL_NAMES} ${PROJECT_NAME} PARENT_SCOPE)

elseif(DEPLOYMENT)
    set(CMAKE_MODULE_PATH "${ROOTDIR}/cmake" ${CMAKE_MODULE_PATH})
    find_package(MinuszeroPerceptionROSTemplate REQUIRED)
    find_package(commons REQUIRED)
    set(SOURCES
        src/freespace.cpp               
        ecal/freespace_node_ecal.cpp
        ecal/run_freespace_ecal.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../node.cpp
    )
    add_executable(${PROJECT_NAME} ${SOURCES})
    message("Project Source Directroy: ${CMAKE_SOURCE_DIR}")
    target_include_directories(
        ${PROJECT_NAME} 
        PUBLIC
        ${OpenCV_INCLUDE_DIRS}
        ${CUDAToolkit_INCLUDE_DIRS}
        ${TensorRT_INCLUDE_DIRS}
        ${xtensor_INCLUDE_DIRS}
        ${TensorRT_DIR}/samples/common
        ${MODULE_INCLUDE_DIRS}
        ${commons_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/ecal
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
        ${TORCH_INCLUDE_DIRS}
        # ${perception_proto_INCLUDE_DIR}
        ${PERCEPTION_DATA_STRUCTURES_INCLUDE_DIR}
        # ${Protobuf_INCLUDE_DIRS}
        # ${Boost_INCLUDE_DIRS}
    )
    target_link_directories(
        ${PROJECT_NAME}
        PUBLIC
        ${CUDAToolkit_LIBRARY_DIR}
        # ${Protobuf_LIBRARIES_DIRS}
    )
    set(LIBRARIES
        cuda-utils
        dlRF
        # datastructures
        eCAL::core
        commons::commons
        # datastructures::perception
    )
    target_link_libraries(
        ${PROJECT_NAME}
        PRIVATE 
        ${TORCH_LIBRARIES} 
        ${OpenCV_LIBS} 
        ${TensorRT_LIBRARIES}
        xtensor
        ${LIBRARIES} 
        ${MODULE_INSTALL_NAMES}
        # ${proto_LIBRARY}
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
    set(INSTALL_NAMES ${INSTALL_NAMES} ${PROJECT_NAME} PARENT_SCOPE)
endif()