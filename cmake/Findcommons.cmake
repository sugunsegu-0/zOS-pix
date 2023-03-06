
set(_COMMONS_PATHS
  ${ROOTDIR}/src/commons
  ${ROOTDIR}/src/commons/build
  ${ROOTDIR}/src/commons/IPC
  ${ROOTDIR}/BUILD/commons
)

# message(${_DL_RUNTIME_FRAMEWORK_PATHS})

find_path(commons_INCLUDE_DIR
  NAMES commons.hpp render/GLrender.hpp jetson-utils/commandLine.h jetson-utils/video/videoOptions.h jetson-utils/video/videoOutput.h
  PATHS ${_COMMONS_PATHS}
)

message( ${commons_INCLUDE_DIR})

set(_COMMONS_SONAME
  "${CMAKE_SHARED_LIBRARY_PREFIX}commons${CMAKE_SHARED_LIBRARY_SUFFIX}"
)

message( ${commons_INCLUDE_DIR})

find_library(commons_LIBRARY
  NAMES "${_COMMONS_SONAME}"
  PATHS ${_COMMONS_PATHS}
  PATH_SUFFIXES "${CMAKE_LIBRARY_ARCHITECTURE}"
)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(COMMONS
  FOUND_VAR COMMONS_FOUND
  REQUIRED_VARS commons_INCLUDE_DIR commons_LIBRARY
)

if(COMMONS_FOUND)
  set(commons_INCLUDE_DIR "${commons_INCLUDE_DIR}")
  set(commons_LIBRARY "${commons_LIBRARY}")

  mark_as_advanced(commons_INCLUDE_DIR commons_LIBRARY)

  if(NOT TARGET  commons::commons)
    add_library( commons::commons SHARED IMPORTED)
    set_target_properties( commons::commons PROPERTIES
      IMPORTED_LOCATION "${commons_LIBRARY}"
      PUBLIC_INCLUDE_DIRECTORIES "${commons_INCLUDE_DIR}"
      IMPORTED_SONAME "${_COMMONS_SONAME}"
    )
  endif()
endif()

unset(_COMMONS_SONAME)
