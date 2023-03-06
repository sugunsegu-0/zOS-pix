
set(_CUDA_UTILS_PATHS
  ${ROOTDIR}/src/perception/cuda-utils
  ${ROOTDIR}/src/perception/cuda-utils/build
  ${ROOTDIR}/src/perception/cuda-utils/include
)

message(_CUDA_UTILS_PATH)

# message(${_CUDA_UTILS_PATHS})

find_path(cudautils_INCLUDE_DIR
  NAMES perception_kernels.hpp
  PATHS ${_CUDA_UTILS_PATHS}
)

set(_CUDA_UTILS_SONAME
  "${CMAKE_SHARED_LIBRARY_PREFIX}cuda-utils${CMAKE_SHARED_LIBRARY_SUFFIX}"
)

message(${cudautils_INCLUDE_DIR})

find_library(cudautils_LIBRARY
  NAMES "${_CUDA_UTILS_SONAME}"
  PATHS ${_CUDA_UTILS_PATHS}
  PATH_SUFFIXES "${CMAKE_LIBRARY_ARCHITECTURE}"
)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(CUDA_UTILS
  FOUND_VAR CUDA_UTILS_FOUND
  REQUIRED_VARS cudautils_INCLUDE_DIR cudautils_LIBRARY
)

if(CUDA_UTILS_FOUND)
  set(cudautils_INCLUDE_DIR "${cudautils_INCLUDE_DIR}")
  set(cudautils_LIBRARY "${cudautils_LIBRARY}")

  mark_as_advanced(cudautils_INCLUDE_DIR cudautils_LIBRARY)

  if(NOT TARGET cudautils::cudautils)
    add_library(cudautils::cudautils SHARED IMPORTED)
    set_target_properties(cudautils::cudautils PROPERTIES
      IMPORTED_LOCATION "${cudautils_LIBRARY}"
      PUBLIC_INCLUDE_DIRECTORIES "${cudautils_INCLUDE_DIR}"
      IMPORTED_SONAME "${_CUDA_UTILS_SONAME}"
    )
  endif()
endif()

unset(_CUDA_UTILS_SONAME)
