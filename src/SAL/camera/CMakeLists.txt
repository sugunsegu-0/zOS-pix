cmake_minimum_required(VERSION 3.16)
project(e-multicam)

find_package(PkgConfig REQUIRED)
find_package(eCAL REQUIRED)

pkg_check_modules(gtk3 REQUIRED IMPORTED_TARGET gtk+-3.0)
pkg_check_modules(PKG_CONFIG_MODULES gstreamer-1.0 gstreamer-video-1.0 gstreamer-app-1.0 x11)
pkg_search_module(gstreamer REQUIRED IMPORTED_TARGET gstreamer-1.0>=1.4)
pkg_search_module(gstreamer-sdp REQUIRED IMPORTED_TARGET gstreamer-sdp-1.0>=1.4)
pkg_search_module(gstreamer-app REQUIRED IMPORTED_TARGET gstreamer-app-1.0>=1.4)
pkg_search_module(gstreamer-video REQUIRED IMPORTED_TARGET gstreamer-video-1.0>=1.4)

set(CMAKE_C_COMPILER ${CROSS_COMPILE}gcc)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -Wall -pedantic -DVERSION=\\\"${VERSION}\\\"")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lpthread")
set(CMAKE_CXX_STANDARD 11)

file(GLOB_RECURSE SRC_FILES "*.c")

add_executable(e-multicam ${SRC_FILES})
target_include_directories(e-multicam
    PUBLIC
    ${PKG_CONFIG_MODULES_INCLUDE_DIRS}
)
target_link_libraries(e-multicam pthread m jpeg ${PKG_CONFIG_MODULES_LDFLAGS}
    eCAL::core_c
    PkgConfig::gtk3
    PkgConfig::gstreamer
    PkgConfig::gstreamer-sdp
    PkgConfig::gstreamer-app
    PkgConfig::gstreamer-video
    )

# install(TARGETS e-multicam DESTINATION ./install)
