cmake_minimum_required(VERSION 3.5)
project(mapping)

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

find_package(MinuszeroMappingTemplate REQUIRED)

set(SOURCES
  src/main.cpp
)

add_executable(${PROJECT_NAME}
       ${SOURCES}
)

target_include_directories(${PROJECT_NAME}  
  PUBLIC  
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${Boost_INCLUDE_DIRS} 
  ${commons_INCLUDE_DIR}
  ${MAPPING_DATA_STRUCTURES_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
 )

target_link_libraries(${PROJECT_NAME}
  eCAL::core
  # datastructures::perception
  commons::commons
  Boost::serialization
  Boost::iostreams
  Boost::boost
  ${OpenCV_LIBS} 
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

