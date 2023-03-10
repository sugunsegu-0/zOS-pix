cmake_minimum_required(VERSION 3.5)
project(gnss)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_BUILD_TYPE Debug)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(CMAKE_MODULE_PATH "${ROOTDIR}/cmake" ${CMAKE_MODULE_PATH})

message(${CMAKE_MODULE_PATH})

find_package(Eigen3 REQUIRED)
find_package(eCAL REQUIRED)
find_package(Boost REQUIRED serialization iostreams)
find_package(Boost REQUIRED)
find_package(commons REQUIRED)
find_package(GPS REQUIRED)

set(SOURCES
  src/gnss.cpp
  src/libgpsmm.cpp
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
  # ${GPSMM}
  ${GPS_INCLUDE_DIR}
  "/home/mzjet/dev/gpsd-3.25/include"
  "/home/mzjet/work/zOS-pix/src/data-structures/include"

 )
# PROTOBUF_TARGET_CPP(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/proto/ ${protobuf_files})


target_link_libraries(${PROJECT_NAME}
Boost::serialization
Boost::iostreams
Boost::boost
${OpenCV_LIBS} 
${Eigen_LIBRARIES}
eCAL::core
commons::commons
gps::gps
)


install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

