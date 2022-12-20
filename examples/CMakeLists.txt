cmake_minimum_required(VERSION 3.5)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/Modules/")
set(BUILD_FLATBUFFER ON CACHE BOOLEAN "Build flatbuffer versions of libraries")

# Set a default build type if none was specified
set(default_build_type "Release")
if(EXISTS "${CMAKE_SOURCE_DIR}/../.git")
  set(default_build_type "Debug")
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(OSMPVERSION "1.3.0" CACHE STRING "OSMP Version String")

add_subdirectory( open-simulation-interface )

get_directory_property(OSI_VERSION_MAJOR DIRECTORY open-simulation-interface DEFINITION VERSION_MAJOR)
get_directory_property(OSI_VERSION_MINOR DIRECTORY open-simulation-interface DEFINITION VERSION_MINOR)
get_directory_property(OSI_VERSION_PATCH DIRECTORY open-simulation-interface DEFINITION VERSION_PATCH)
set(OSIVERSION "${OSI_VERSION_MAJOR}.${OSI_VERSION_MINOR}.${OSI_VERSION_PATCH}")

include_directories( includes )
if(BUILD_FLATBUFFER)
  add_subdirectory( OSMPDummySensor_flat )
  add_subdirectory( OSMPDummySource_flat )
else()
  add_subdirectory( OSMPDummySensor )
  add_subdirectory( OSMPDummySource )
  add_subdirectory( OSMPCNetworkProxy )
endif()
