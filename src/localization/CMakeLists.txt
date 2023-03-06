cmake_minimum_required(VERSION 3.5)
project(ekf_localization_node)

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


# find_package(MinuszeroMappingTemplate REQUIRED)


find_package(Eigen3 REQUIRED)
find_package(eCAL REQUIRED)
find_package(Boost REQUIRED serialization iostreams)
find_package(Boost REQUIRED)
find_package(datastructures REQUIRED)
find_package(MinuszeroLocalizationROSTemplate  REQUIRED)
set(SOURCES
  src/ekf_localization_node.cpp
  src/ros_filter.cpp
  src/filter_utilities.cpp
  src/filter_base.cpp
  src/ekf.cpp
  src/robot_localization_estimator.cpp
  src/ros_filter.cpp
)

add_executable(${PROJECT_NAME}
       ${SOURCES}
)

target_include_directories(${PROJECT_NAME}  
  PUBLIC  
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS} 
  ${commons_INCLUDE_DIR}
  ${LOCALIZATION_DATA_STRUCTURES_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
 )

target_link_libraries(${PROJECT_NAME}
  ${Eigen_LIBRARIES}
  eCAL::core
  commons::commons
  Boost::serialization
  Boost::iostreams
  Boost::boost
  ${OpenCV_LIBS} 
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

