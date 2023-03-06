# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


# In House Library Addition
find_package(commons REQUIRED)
find_package(datastructures REQUIRED)
find_package(OpenCV REQUIRED)
#---------------------- iceoryx + ecal --------------------
if(DEPLOYMENT)
    include(GNUInstallDirs)
    find_package(Boost REQUIRED serialization iostreams)
    find_package(Boost REQUIRED )
    # find_package(Protobuf REQUIRED)
    # find_package(Protos REQUIRED)
    find_package(eCAL REQUIRED)
endif()
#---------------------- iceoryx --------------------

#---------------------- ros --------------------
#---------------------- ros --------------------