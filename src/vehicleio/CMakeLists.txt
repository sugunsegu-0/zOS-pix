#  TODO add directories

cmake_minimum_required(VERSION 3.5)
project(vehicleio)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_BUILD_TYPE Debug)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(CMAKE_MODULE_PATH "${ROOTDIR}/cmake" ${CMAKE_MODULE_PATH})

# find packages
find_package(eCAL REQUIRED)
find_package(commons REQUIRED)
find_package(OpenCV REQUIRED)
find_package(datastructures REQUIRED)
find_package(Boost REQUIRED serialization iostreams)
find_package(Boost REQUIRED)
find_package(socketcan_cpp)

set(SOURCES
  src/main.cpp
  src/vehicleio.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} 
    PUBLIC
    ${commons_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${CONTROL_DATA_STRUCTURES_INCLUDE_DIR}
    ${VEHICLE_DATA_STRUCTURES_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS} 
    )

target_link_libraries(${PROJECT_NAME}  
    PUBLIC
    eCAL::core
    commons::commons
    ${OpenCV_LIBS} 
    Boost::serialization
    Boost::iostreams
    Boost::boost
    pthread
    ::socketcan_cpp
    )

install(TARGETS
    ${INSTALL_NAMES}
    DESTINATION lib/${PROJECT_NAME})
