# This file provides Findmscl, a CMake module that searches for the MSCL library.
#
# To use this module, add the following lines to your CMakeLists.txt file:
#
#   set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/path/to/Findmscl.cmake")
#   find_package(mscl REQUIRED)
#   target_link_libraries(your_executable mscl::mscl)

# Define a helper function to locate the MSCL library
function(find_mscl)
  find_library(MSCL_LIBRARY 
               NAMES mscl
               PATHS /usr/share/c++-mscl)
  find_path(MSCL_INCLUDE_DIR 
            NAMES mscl/mscl.h
            PATHS /usr/share/c++-mscl/source)

  if (MSCL_LIBRARY AND MSCL_INCLUDE_DIR)
    set(MSCL_FOUND TRUE)
  endif()

  if (NOT MSCL_FOUND)
    message(FATAL_ERROR "Could not find MSCL library")
  endif()

  set(MSCL_LIBRARIES ${MSCL_LIBRARY})
  set(MSCL_INCLUDE_DIRS ${MSCL_INCLUDE_DIR})
  set(MSCL_INCLUDE_DIRS ${MSCL_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
endfunction()

# Call the helper function to locate the MSCL library
find_mscl()

# Set up the MSCL target for use in downstream projects
add_library(mscl INTERFACE)
target_include_directories(mscl INTERFACE ${MSCL_INCLUDE_DIRS})
target_link_libraries(mscl INTERFACE ${MSCL_LIBRARIES})
