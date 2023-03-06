# This module cannot run independently
# It is meant to be a headerfile api ONLY.

cmake_minimum_required(VERSION 3.16)
project(object-detection CUDA CXX C)

if(RENDER_PERCEPTION_OBJECT_DETECTION)
    # set(GLFW_DIR /usr/lib/x86_64-linux-gnu/cmake/glfw3)
    # set(GLFW_LIB /usr/lib/x86_64-linux-gnu)
    # set(GLEW_DIR /usr/share/cmake-3.16/Modules)
    find_package(OpenGL REQUIRED)
    find_package(GLUT REQUIRED)
    find_package(glfw3 REQUIRED)
    find_package(GLEW REQUIRED)

    include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )
    link_libraries( GLEW::GLEW ${OPENGL_LIBRARIES} glfw ${GLUT_LIBRARY} )
    add_definitions(-DRENDER_PERCEPTION_OBJECT_DETECTION)
endif()



# find_package(Python REQUIRED)
# find_package(TorchVision REQUIRED)

if(CORE_DEBUG)
    set(SOURCES
        src/object-detection.cpp
    )
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
        ${CUDA_LIBRARIES} 
        ${OpenCV_LIBS} 
        ${TensorRT_LIBRARIES}
        ${LIBRARIES}
        ${DEPENDENCY_LIBRARIES}
    )
    set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
    set(DEPENDENCY_LIBRARIES ${DEPENDENCY_LIBRARIES} ${PROJECT_NAME} PARENT_SCOPE)

elseif(CORE_DEVELOPMENT)
    set(CMAKE_MODULE_PATH "${ROOTDIR}/cmake" ${CMAKE_MODULE_PATH})
    find_package(MinuszeroPerceptionROSTemplate REQUIRED)
    set(SOURCES
        src/object-detection.cpp
        ros/object_detection_node_ros.cpp
        ros/run_object_detection_ros.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../node.cpp
    )
    set(LIBRARIES
        dlRF
        # datastructures::perception
    )
    add_executable(${PROJECT_NAME} ${SOURCES})
    message("Project Directroy: ${CMAKE_CURRENT_SOURCE_DIR}")
    target_include_directories(
        ${PROJECT_NAME} 
        PUBLIC
        ${OpenCV_INCLUDE_DIRS}
        ${CUDAToolkit_INCLUDE_DIRS}
        ${TensorRT_INCLUDE_DIRS}
        ${TensorRT_DIR}/samples/common
        ${MODULE_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/ros
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
        ${TORCH_INCLUDE_DIRS}
        #${TorchVision_INCLUDE_DIR}
        ${PERCEPTION_DATA_STRUCTURES_INCLUDE_DIR}

        # ros include dirs
        ${Boost_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        ${sensor_INCLUDE_DIRS}
        ${minuszero_msgs_INCLUDE_DIRS}
        ${cv_bridge_INCLUDE_DIRS}
        ${message_filters_INCLUDE_DIRS}
        ${image_transport_INCLUDE_DIRS}
    )
    target_link_directories(
        ${PROJECT_NAME}
        PUBLIC
        ${CUDAToolkit_LIBRARY_DIR}
    )
    target_link_libraries(
        ${PROJECT_NAME}
        PUBLIC 
        ${TORCH_LIBRARIES} 
        ${OpenCV_LIBS} 
        ${TensorRT_LIBRARIES}
        ${LIBRARIES}
        # TorchVision::TorchVision

        # ros libraries
        ${MODULE_INSTALL_NAMES}
        ${rclcpp_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        ${minuszero_msgs_LIBRARIES}
        ${cv_bridge_LIBRARIES}
        ${message_filters_LIBRARIES}
    )
    set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
    set(INSTALL_NAMES ${INSTALL_NAMES} ${PROJECT_NAME} PARENT_SCOPE)
    
elseif(DEPLOYMENT)

    set(CMAKE_MODULE_PATH "${ROOTDIR}/cmake" ${CMAKE_MODULE_PATH})
    find_package(MinuszeroPerceptionROSTemplate REQUIRED)
    find_package(commons REQUIRED)
    set(SOURCES
        src/object-detection.cpp
        ecal/object_detection_node_ecal.cpp
        ecal/run_object_detection_ecal.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../node.cpp
    )
    add_executable(${PROJECT_NAME} ${SOURCES})
    message("Project Directroy: ${CMAKE_CURRENT_SOURCE_DIR}")
    message("${commons_INCLUDE_DIR}")
    target_include_directories(
        ${PROJECT_NAME} 
        PUBLIC
        ${OpenCV_INCLUDE_DIRS}
        ${CUDAToolkit_INCLUDE_DIRS}
        ${TensorRT_INCLUDE_DIRS}
        ${TensorRT_DIR}/samples/common
        ${MODULE_INCLUDE_DIRS}
        ${commons_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/ecal
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
        ${TORCH_INCLUDE_DIRS}
        # ${TorchVision_INCLUDE_DIR}
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

    message("asdasdasdd ${TORCH_LIBRARIES}")
    target_link_libraries(
        ${PROJECT_NAME}
        PRIVATE 
        ${TORCH_LIBRARIES} 
        ${OpenCV_LIBS} 
        ${TensorRT_LIBRARIES}
        CUDA::toolkit
        CUDA::npps
        CUDA::nppc
        CUDA::nppicc
        dlRF
        cuda-utils
        commons::commons
        eCAL::core
        Boost::serialization
        Boost::boost
        Boost::iostreams
        # TorchVision::TorchVision
        ${MODULE_INSTALL_NAMES}
    )
    set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
    set(INSTALL_NAMES ${INSTALL_NAMES} ${PROJECT_NAME} PARENT_SCOPE)
endif()