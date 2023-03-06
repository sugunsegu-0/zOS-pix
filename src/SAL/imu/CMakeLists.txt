cmake_minimum_required(VERSION 3.16)
project(imu)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#If compiler is clang instead of gcc.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ROOTDIR /home/minuszero-ams/dev/zOS)

set(CMAKE_MODULE_PATH "${ROOTDIR}/cmake" ${CMAKE_MODULE_PATH})
message(${CMAKE_MODULE_PATH})

find_package(mscl REQUIRED)
# find_package(Boost REQUIRED)
find_package(eCAL REQUIRED)
# find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED serialization iostreams)
find_package(Boost REQUIRED)
find_package(commons REQUIRED)
find_package(datastructures REQUIRED)

set(SOURCES
    src/imu_msg_struct.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${MSCL_INCLUDE_DIR}
    ${MSCL_INCLUDE_DIR}/../Boost/include
    ${Boost_INCLUDE_DIRS} 
    ${commons_INCLUDE_DIR}
    ${LOCALIZATION_DATA_STRUCTURES_INCLUDE_DIR}

)

# target_link_directories(
#     ${PROJECT_NAME}
#     PUBLIC
    
# )

target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
    eCAL::core
    Boost::serialization
    Boost::boost
    Boost::iostreams
    ${MSCL_LIBRARY}
    commons::commons
)


install(TARGETS
        ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})