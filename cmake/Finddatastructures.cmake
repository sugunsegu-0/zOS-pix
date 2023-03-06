set(_data_structures_PATHS
    ${ROOTDIR}/src/data-structures
    ${ROOTDIR}/src/data-structures/include
)

include(FindPackageHandleStandardArgs)

## PERCEPTION DATA STRUCTURES
find_path(PERCEPTION_DATA_STRUCTURES_INCLUDE_DIR
    NAMES perception/perceptionDS.hpp
    PATHS ${_data_structures_PATHS}
)
find_package_handle_standard_args(PERCEPTION_DATA_STRUCTURES
    FOUND_VAR PERCEPTION_DATA_STRUCTURES_FOUND
    REQUIRED_VARS PERCEPTION_DATA_STRUCTURES_INCLUDE_DIR
)
if(PERCEPTION_DATA_STRUCTURES_FOUND)
    set(PERCEPTION_DATA_STRUCTURES_INCLUDE_DIR "${PERCEPTION_DATA_STRUCTURES_INCLUDE_DIR}")
    mark_as_advanced(PERCEPTION_DATA_STRUCTURES_INCLUDE_DIR)
    if(NOT TARGET datastructures::perception)
        add_library(datastructures::perception SHARED IMPORTED)
        set_target_properties(datastructures::perception PROPERTIES
            PUBLIC_INCLUDE_DIRECTORIES "${PERCEPTION_DATA_STRUCTURES_INCLUDE_DIR}"
        )
    endif()
endif()

# MAPPING DATA STRUCTURES
find_path(MAPPING_DATA_STRUCTURES_INCLUDE_DIR
    NAMES mapping/mappingDS.hpp
    PATHS ${_data_structures_PATHS}
)
find_package_handle_standard_args(MAPPING_DATA_STRUCTURES
    FOUND_VAR MAPPING_DATA_STRUCTURES_FOUND
    REQUIRED_VARS MAPPING_DATA_STRUCTURES_INCLUDE_DIR
)
if(MAPPING_DATA_STRUCTURES_FOUND)
    set(MAPPING_DATA_STRUCTURES_INCLUDE_DIR "${MAPPING_DATA_STRUCTURES_INCLUDE_DIR}")
    mark_as_advanced(MAPPING_DATA_STRUCTURES_INCLUDE_DIR)
    if(NOT TARGET datastructures::mapping)
        add_library(datastructures::mapping SHARED IMPORTED)
        set_target_properties(datastructures::mapping PROPERTIES
            PUBLIC_INCLUDE_DIRECTORIES "${MAPPING_DATA_STRUCTURES_INCLUDE_DIR}"
        )
    endif()
endif()

# MOTION-PLANNING DATA STRUCTURES

find_path(MOTION_PLANNING_DATA_STRUCTURES_INCLUDE_DIR
    NAMES motion-planning/motion-planningDS.hpp
    PATHS ${_data_structures_PATHS}
)
find_package_handle_standard_args(MOTION_PLANNING_DATA_STRUCTURES
    FOUND_VAR MOTION_PLANNING_DATA_STRUCTURES_FOUND
    REQUIRED_VARS MOTION_PLANNING_DATA_STRUCTURES_INCLUDE_DIR
)
if(MOTION_PLANNING_DATA_STRUCTURES_FOUND)
    set(MOTION_PLANNING_DATA_STRUCTURES_INCLUDE_DIR "${MOTION_PLANNING_DATA_STRUCTURES_INCLUDE_DIR}")
    mark_as_advanced(MOTION_PLANNING_DATA_STRUCTURES_INCLUDE_DIR)
    if(NOT TARGET datastructures::motion-planning)
        add_library(datastructures::motion-planning SHARED IMPORTED)
        set_target_properties(datastructures::motion-planning PROPERTIES
            PUBLIC_INCLUDE_DIRECTORIES "${MOTION_PLANNING_DATA_STRUCTURES_INCLUDE_DIR}"
        )
    endif()
endif()

# LOCALIZATION DATA STRUCTURES
find_path(LOCALIZATION_DATA_STRUCTURES_INCLUDE_DIR
    NAMES localization/localizationDS.hpp
    PATHS ${_data_structures_PATHS}
)
find_package_handle_standard_args(LOCALIZATION_DATA_STRUCTURES
    FOUND_VAR LOCALIZATION_DATA_STRUCTURES_FOUND
    REQUIRED_VARS LOCALIZATION_DATA_STRUCTURES_INCLUDE_DIR
)
if(LOCALIZATION_DATA_STRUCTURES_FOUND)
    set(LOCALIZATION_DATA_STRUCTURES_INCLUDE_DIR "${LOCALIZATION_DATA_STRUCTURES_INCLUDE_DIR}")
    mark_as_advanced(LOCALIZATION_DATA_STRUCTURES_INCLUDE_DIR)
    if(NOT TARGET datastructures::localization)
        add_library(datastructures::localization SHARED IMPORTED)
        set_target_properties(datastructures::localization PROPERTIES
            PUBLIC_INCLUDE_DIRECTORIES "${LOCALIZATION_DATA_STRUCTURES_INCLUDE_DIR}"
        )
    endif()
endif()
find_path(DRIVERS_DATA_STRUCTURES_INCLUDE_DIR
    NAMES drivers/gnss_data.hpp
    PATHS ${_data_structures_PATHS}
)
find_package_handle_standard_args(DRIVERS_DATA_STRUCTURES_INCLUDE_DIR
    FOUND_VAR DRIVERS_DATA_STRUCTURES_INCLUDE_DIR_FOUND
    REQUIRED_VARS DRIVERS_DATA_STRUCTURES_INCLUDE_DIR
)
if(DRIVERS_DATA_STRUCTURES_INCLUDE_DIR_FOUND)
    set(DRIVERS_DATA_STRUCTURES_INCLUDE_DIR "${DRIVERS_DATA_STRUCTURES_INCLUDE_DIR}")
    mark_as_advanced(DRIVERS_DATA_STRUCTURES_INCLUDE_DIR)
    if(NOT TARGET datastructures::drivers)
        add_library(datastructures::drivers SHARED IMPORTED)
        set_target_properties(datastructures::drivers PROPERTIES
            PUBLIC_INCLUDE_DIRECTORIES "${DRIVERS_DATA_STRUCTURES_INCLUDE_DIR}"
        )
    endif()
endif()
## CONTROL DATA STRUCTURES
find_path(CONTROL_DATA_STRUCTURES_INCLUDE_DIR
    NAMES control/control-DS.hpp
    PATHS ${_data_structures_PATHS}
)
find_package_handle_standard_args(CONTROL_DATA_STRUCTURES
    FOUND_VAR CONTROL_DATA_STRUCTURES_FOUND
    REQUIRED_VARS CONTROL_DATA_STRUCTURES_INCLUDE_DIR
)
if(CONTROL_DATA_STRUCTURES_FOUND)
    set(CONTROL_DATA_STRUCTURES_INCLUDE_DIR "${CONTROL_DATA_STRUCTURES_INCLUDE_DIR}")
    mark_as_advanced(CONTROL_DATA_STRUCTURES_INCLUDE_DIR)
    if(NOT TARGET datastructures::control)
        add_library(datastructures::control SHARED IMPORTED)
        set_target_properties(datastructures::control PROPERTIES
            PUBLIC_INCLUDE_DIRECTORIES "${CONTROL_DATA_STRUCTURES_INCLUDE_DIR}"
        )
    endif()
endif()
## VEHICLE DATA STRUCTURES
find_path(VEHICLE_DATA_STRUCTURES_INCLUDE_DIR
    NAMES vehicleio/vehicle_DS.hpp
    PATHS ${_data_structures_PATHS}
)
find_package_handle_standard_args(VEHICLE_DATA_STRUCTURES
    FOUND_VAR VEHICLE_DATA_STRUCTURES_FOUND
    REQUIRED_VARS VEHICLE_DATA_STRUCTURES_INCLUDE_DIR
)
if(VEHICLE_DATA_STRUCTURES_FOUND)
    set(VEHICLE_DATA_STRUCTURES_INCLUDE_DIR "${VEHICLE_DATA_STRUCTURES_INCLUDE_DIR}")
    mark_as_advanced(VEHICLE_DATA_STRUCTURES_INCLUDE_DIR)
    if(NOT TARGET datastructures::vehicle)
        add_library(datastructures::vehicle SHARED IMPORTED)
        set_target_properties(datastructures::vehicle PROPERTIES
            PUBLIC_INCLUDE_DIRECTORIES "${VEHICLE_DATA_STRUCTURES_INCLUDE_DIR}"
        )
    endif()
endif()