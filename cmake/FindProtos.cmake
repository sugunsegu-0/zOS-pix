
set(_PROTO_PATHS
${ROOTDIR}/src/protos
${ROOTDIR}/src/protos/build
)

find_path(perception_proto_INCLUDE_DIR
NAMES object_detection_DS.pb.h segmentation_DS.pb.h lane_estimation_DS.pb.h freespace_DS.pb.h
PATHS ${_PROTO_PATHS}
)

set(_PROTO_SONAME
  "${CMAKE_SHARED_LIBRARY_PREFIX}proto${CMAKE_SHARED_LIBRARY_SUFFIX}"
)

find_library(proto_LIBRARY
NAMES "${_PROTO_SONAME}"
PATHS ${_PROTO_PATHS}
PATH_SUFFIXES "${CMAKE_LIBRARY_ARCHITECTURE}"
)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(PROTO
FOUND_VAR PROTO_FOUND
REQUIRED_VARS perception_proto_INCLUDE_DIR proto_LIBRARY
)

if(PROTO_FOUND)
set(perception_proto_INCLUDE_DIR "${perception_proto_INCLUDE_DIR}")
set(proto_LIBRARY "${proto_LIBRARY}")

mark_as_advanced(perception_proto_INCLUDE_DIR proto_LIBRARY)

if(NOT TARGET proto::proto)
  add_library(proto::proto SHARED IMPORTED)
  set_target_properties(proto::proto PROPERTIES
    IMPORTED_LOCATION "${proto_LIBRARY}"
    PUBLIC_INCLUDE_DIRECTORIES "${perception_proto_INCLUDE_DIR}"
    IMPORTED_SONAME "${_PROTO_SONAME}"
  )
endif()
endif()

unset(_PROTO_SONAME)
