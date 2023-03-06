# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
#-------------------------------------------------------------------------------
# Set CUDA_DIR
#-------------------------------------------------------------------------------
if (DEFINED CUDA_DIR)
    if((DEFINED CUDA_TOOLKIT_ROOT_DIR) AND (NOT CUDA_TOOLKIT_ROOT_DIR STREQUAL CUDA_DIR))
        message(FATAL_ERROR "Cannot set both CUDA_DIR and (legacy) CUDA_TOOLKIT_ROOT_DIR")
    endif()
elseif (DEFINED CUDA_TOOLKIT_ROOT_DIR)
    message(WARNING "Please set CUDA_DIR instead of (legacy) CUDA_TOOLKIT_ROOT_DIR")
    set(CUDA_DIR  ${CUDA_TOOLKIT_ROOT_DIR} CACHE PATH "CUDA Toolkit location.")
else()
    set(CUDA_DIR  "/usr/local/cuda/" CACHE PATH "CUDA Toolkit location.")
endif()
if(NOT CMAKE_CUDA_COMPILER)
    set(CMAKE_CUDA_COMPILER "${CUDA_DIR}/bin/nvcc")
endif()
set(CMAKE_CUDA_HOST_COMPILER ${CMAKE_CXX_COMPILER})
enable_language(CUDA)

set(CMAKE_CUDA_STANDARD 11)
set(CMAKE_C_STANDARD 99)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Ofast -DNDEBUG ${TORCH_CXX_FLAGS}")
# message(${CUDA_NVCC_FLAGS})
message(ROOT)
# Use the correct version of CUDA
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)


set(CUDA_LINK_LIBRARIES_KEYWORD PUBLIC)
set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()


# find dependencies
find_package(CUDAToolkit REQUIRED)
message(STATUS "Found CUDA ${CUDA_VERSION_STRING} at ${CUDA_TOOLKIT_ROOT_DIR}")
find_package(OpenCV REQUIRED)
message("${TensorRT_DIR}")
find_package(TensorRT REQUIRED)
message("${TensorRT_DIR}")
find_package(xtensor CONFIG REQUIRED)
# find_package(Thrust REQUIRED CONFIG)
find_package(Torch CONFIG REQUIRED)
# if(NOT THRUST_FOUND)
#     thrust_create_target(Thrust)
# endif()

# In House Library Addition
find_package(commons REQUIRED)
find_package(datastructures REQUIRED)

#---------------------- iceoryx + ecal --------------------
if(DEPLOYMENT)
    include(GNUInstallDirs)
    find_package(Boost REQUIRED serialization iostreams)
    find_package(Boost REQUIRED )
    find_package(eCAL REQUIRED)
endif()
#---------------------- iceoryx --------------------

#---------------------- ros --------------------
if(CORE_DEVELOPMENT)
    find_package(ament_cmake_auto REQUIRED)
    ament_auto_find_build_dependencies()
    set(sensor_msgs_DIR /opt/ros/foxy/share/sensor_msgs)

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(vision_msgs REQUIRED)
    find_package(minuszero_msgs REQUIRED)
    find_package(cv_bridge REQUIRED)
    find_package(image_transport REQUIRED)
    find_package(message_filters REQUIRED)
endif()
#---------------------- ros --------------------