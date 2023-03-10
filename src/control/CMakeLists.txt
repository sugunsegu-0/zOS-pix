cmake_minimum_required(VERSION 3.5)
project(control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_BUILD_TYPE Debug)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
set(CMAKE_MODULE_PATH "${ROOTDIR}/cmake" ${CMAKE_MODULE_PATH})

message(${CMAKE_MODULE_PATH})

find_package(Matplot++ REQUIRED)
find_package(commons REQUIRED)
find_package(datastructures REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED serialization iostreams)
find_package(Boost REQUIRED )
# find_package(Protobuf REQUIRED)
# find_package(Protos REQUIRED)
find_package(eCAL REQUIRED)

set(SOURCES
  src/control_w_waypoint_collector.cpp
)

add_executable(${PROJECT_NAME}
       ${SOURCES}
)

target_include_directories(${PROJECT_NAME}  
  PUBLIC  
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${Boost_INCLUDE_DIRS} 
  ${commons_INCLUDE_DIR}
  ${MOTION_PLANNING_DATA_STRUCTURES_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
 )
# PROTOBUF_TARGET_CPP(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/proto/ ${protobuf_files})

target_link_libraries(${PROJECT_NAME}
  
    eCAL::core
    Matplot++::matplot
#   datastructures::motion-planning
  commons::commons
  # protobuf::libprotobuf
  Boost::serialization
  Boost::iostreams
  Boost::boost
  ${OpenCV_LIBS} 
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

