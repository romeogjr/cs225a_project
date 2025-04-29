cmake_minimum_required(VERSION 3.10)

project(SAI-INTERFACES)
set(PROJECT_VERSION 0.1.0)

option(BUILD_EXAMPLES "Build examples" ON)

set(CMAKE_CXX_FLAGS "-std=c++20 -I/usr/include -I/usr/local/include -fPIC")
if(${CMAKE_SYSTEM_NAME} MATCHES Darwin)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/opt/homebrew/include")
endif()

# set default build to release
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release)
endif()

# include Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# include Chai3d
find_package(CHAI3D REQUIRED)
include_directories(${CHAI3D_INCLUDE_DIRS})
add_definitions(${CHAI3D_DEFINITIONS})
link_directories(${CHAI3D_LIBRARY_DIRS})

# * SAI-COMMON
find_package(SAI-COMMON REQUIRED)
include_directories(${SAI-COMMON_INCLUDE_DIRS})

# * SAI-URDF
find_package(SAI-URDF REQUIRED)
include_directories(${SAI-URDF_INCLUDE_DIRS})

# * SAI-MODEL
find_package(SAI-MODEL REQUIRED)
include_directories(${SAI-MODEL_INCLUDE_DIRS})

# * SAI-SIMULATION
find_package(SAI-SIMULATION REQUIRED)
include_directories(${SAI-SIMULATION_INCLUDE_DIRS})
add_definitions(${SAI-SIMULATION_DEFINITIONS})

# * SAI-GRAPHICS
find_package(SAI-GRAPHICS REQUIRED)
include_directories(${SAI-GRAPHICS_INCLUDE_DIRS})
add_definitions(${SAI-GRAPHICS_DEFINITIONS})

# * SAI-PRIMITIVES
find_package(SAI-PRIMITIVES REQUIRED)
include_directories(${SAI-PRIMITIVES_INCLUDE_DIRS})

# add redis interface classes
set(SAI_INTERFACES_SOURCE
    ${PROJECT_SOURCE_DIR}/src/helpers/ConfigParserHelpers.cpp
    ${PROJECT_SOURCE_DIR}/src/simviz/SimVizRedisInterface.cpp
    ${PROJECT_SOURCE_DIR}/src/simviz/SimVizConfigParser.cpp
    ${PROJECT_SOURCE_DIR}/src/controller/RobotControllerRedisInterface.cpp
    ${PROJECT_SOURCE_DIR}/src/controller/RobotControllerConfigParser.cpp
    ${PROJECT_SOURCE_DIR}/src/controller/HapticDeviceControllerRedisInterface.cpp
    ${PROJECT_SOURCE_DIR}/src/controller/HapticDeviceControllerConfigParser.cpp
    ${PROJECT_SOURCE_DIR}/src/MainRedisInterface.cpp)

set(SAI-INTERFACES_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/src/third_party/glaze/include)
include_directories(${SAI-INTERFACES_INCLUDE_DIRS})

set(SAI-INTERFACES_UI_DIR ${PROJECT_SOURCE_DIR}/ui)
add_definitions(-DUI_FOLDER="${SAI-INTERFACES_UI_DIR}")

# Create the library
add_library(sai-interfaces STATIC ${SAI_INTERFACES_SOURCE})

# Ensure Glaze include directories are part of the target
target_include_directories(
  sai-interfaces
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
         $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/third_party/glaze/include>
         $<INSTALL_INTERFACE:include>)

set(SAI-INTERFACES_LIBRARIES
    sai-interfaces
    ${SAI-PRIMITIVES_LIBRARIES}
    ${SAI-SIMULATION_LIBRARIES}
    ${SAI-GRAPHICS_LIBRARIES}
    ${CHAI3D_LIBRARIES}
    ${SAI-MODEL_LIBRARIES}
    ${SAI-URDF_LIBRARIES}
    ${SAI-COMMON_LIBRARIES})

# add examples
if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

#
# export package
#
set(SAI-INTERFACES_UI_DIR ${PROJECT_SOURCE_DIR}/ui)

# export package for use from the build tree
export(TARGETS sai-interfaces
       FILE ${PROJECT_BINARY_DIR}/SAI-INTERFACESTargets.cmake)
export(PACKAGE SAI-INTERFACES)

# configure package
set(CONF_INCLUDE_DIRS ${SAI-INTERFACES_INCLUDE_DIRS})
set(CONF_LIBRARIES sai-interfaces)

# create package configuration and version files
configure_file(SAI-INTERFACESConfig.cmake.in
               "${PROJECT_BINARY_DIR}/SAI-INTERFACESConfig.cmake" @ONLY)
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/SAI-INTERFACESConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)
