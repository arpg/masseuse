
project(Masseuse)
cmake_minimum_required(VERSION 2.8)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules/")

string(TOLOWER ${PROJECT_NAME} LIBRARY_NAME)

if(POLICY CMP0042)
    cmake_policy(SET CMP0042 NEW) # MACOSX_RPATH
endif()

include(install_package)
include(SetPlatformVars)

set(Masseuse_VERSION_MAJOR 0)
set(Masseuse_VERSION_MINOR 1)
set(PACKAGE_VERSION ${Masseuse_VERSION_MAJOR}.${Masseuse_VERSION_MINOR})
set(Masseuse_VERSION ${PACKAGE_VERSION})

option(BUILD_APPLICATIONS "Build Applications" ON)
option(BUILD_SHARED_LIBS "Build Shared Library" ON)

set( CMAKE_CXX_FLAGS "-std=c++0x -Wall ${CMAKE_CXX_FLAGS}" )
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

if( NOT CMAKE_BUILD_TYPE )
    set( CMAKE_BUILD_TYPE Release )
endif()

#####################################################################
## Find required packages
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

set( REQUIRED_INCLUDE_DIRS
    ${CERES_INCLUDE_DIR}
    ${Sophus_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    )

set( REQUIRED_LIBRARIES
    ${CERES_LIBRARIES}
    ${Sophus_LIBRARIES}
    )

#####################################################################
## Sources

set( INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/${LIBRARY_NAME} )

set( HEADERS
    ${INC_DIR}/masseuse.h
    ${INC_DIR}/CeresCostFunctions.h
    ${INC_DIR}/LocalParameterizationSE3.h
    ${INC_DIR}/LocalParameterizationSO3.h
    ${INC_DIR}/AutoDiffLocalParamSE3.h
    ${INC_DIR}/Utils.h
    )

set( SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src )

set( SOURCES
    ${SRC_DIR}/masseuse.cpp
    )

include_directories( ${REQUIRED_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include )
add_library( ${LIBRARY_NAME} SHARED ${SOURCES} ${HEADERS} )
target_link_libraries( ${LIBRARY_NAME} ${REQUIRED_LIBRARIES} )
set_target_properties( ${LIBRARY_NAME} PROPERTIES VERSION "${PACKAGE_VERSION}" SOVERSION "${PACKAGE_VERSION}" )


install_package(
    PKG_NAME ${PROJECT_NAME}
    LIB_NAME ${LIBRARY_NAME}
    VERSION ${PACKAGE_VERSION}
    DESCRIPTION "Robust Pose Graph Relaxation library using Ceres-Solver."
    INSTALL_INCLUDE_DIR true
    DESTINATION ${CMAKE_INSTALL_PREFIX}
    INCLUDE_DIRS ${REQUIRED_INCLUDE_DIRS}
    LINK_LIBS ${REQUIRED_LIBRARIES}
    )

####################################################################
if( BUILD_APPLICATIONS )
    set( Masseuse_DIR ${CMAKE_CURRENT_BINARY_DIR} )
    add_subdirectory( applications )
endif()

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)


