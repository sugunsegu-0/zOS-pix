cmake_minimum_required(VERSION 3.16)
project(perception)

set(CMAKE_MODULE_PATH "${ROOTDIR}/cmake" ${CMAKE_MODULE_PATH})

message(${CMAKE_MODULE_PATH})

find_package(MinuszeroPerceptionROSTemplate REQUIRED)

set(INSTALL_NAMES)

if(RENDER_PERCEPTION_OBJECT_DETECTION)
    message("Render Object detection - On")
    add_definitions(-DRENDER_PERCEPTION_OBJECT_DETECTION)
endif()
if(RENDER_PERCEPTION_SEGMENTATION)
    message("Render Segmentation - On")
    add_definitions(-DRENDER_PERCEPTION_SEGMENTATION)
endif()
if(RENDER_PERCEPTION_LANE_ESTIMATION)
    message("Render Lane Estimation - On")
    add_definitions(-DRENDER_PERCEPTION_LANE_ESTIMATION)
endif()
if(RENDER_PERCEPTION_FREESPACE)
    message("Render Freespace - On")
    add_definitions(-DRENDER_PERCEPTION_FREESPACE)
endif()

#################################################################################
#                   BUILDING PERCEPTION DEPENDENCIES                            #
#################################################################################
set(DEPENDENCY_DIRECTORIES
    cuda-utils
    dl-runtime-framework
)
set(DEPENDENCY_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/include
)

set(LIB_INSTALL_DIR
    ${CMAKE_INSTALL_PREFIX}/lib
)

set(INCLUDE_INSTALL_DIR
    ${CMAKE_INSTALL_PREFIX}/include
)

foreach(dependency_icl_dir IN LISTS DEPENDENCY_DIRECTORIES)
    list(APPEND MODULE_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/${dependency_icl_dir}/include)
endforeach()

foreach(dependency IN LISTS DEPENDENCY_DIRECTORIES)
    add_subdirectory(${dependency})
endforeach()

#################################################################################
#                   BUILDING PERCEPTION MODULE                                  #
#################################################################################

set(MODULE_DIRECTORIES
    src/object-detection
    src/segmentation
    src/lane-estimation
    src/freespace        
)

foreach(module_icl_dir IN LISTS MODULE_DIRECTORIES)
    list(APPEND MODULE_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/${module_icl_dir}/include ${CMAKE_SOURCE_DIR}/${module_icl_dir}/component)
endforeach()

add_subdirectory(src/object-detection)
add_subdirectory(src/segmentation)
add_subdirectory(src/lane-estimation)
add_subdirectory(src/freespace)       

if(CORE_DEVELOPMENT)
    ament_export_dependencies(cv_bridge image_transport rclcpp sensor_msgs std_msgs)
endif()

install(TARGETS
    ${INSTALL_NAMES}
    DESTINATION lib/${PROJECT_NAME})
