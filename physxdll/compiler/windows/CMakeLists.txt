cmake_minimum_required(VERSION 3.12)

# This is required to be defined by external callers!
IF(NOT DEFINED PHYSX_ROOT_DIR)
	MESSAGE(FATAL_ERROR "PHYSX_ROOT_DIR variable wasn't set.")
ENDIF()

IF(NOT EXISTS ${PHYSX_ROOT_DIR})
	MESSAGE(FATAL_ERROR "PHYSX_ROOT_DIR variable was invalid.")
ENDIF()

# We define the CXX flags for this CMakeLists and all others that are included afterwards. This is a GLOBAL setting. 
# If/when the solutions go standalone (say, samples and visual tests) - those CMakeLists will need to be fixed.

SET(PHYSX_WARNING_DISABLES "/wd4514 /wd4820 /wd4127 /wd4710 /wd4711 /wd4577 /wd4996")

STRING(TOLOWER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_LOWERCASE)

PROJECT(PhysXDLL C CXX)


# These flags are local to the directory the CMakeLists.txt is in, so don't get carried over to OTHER CMakeLists.txt (thus the CACHE variables above)
SET(PHYSXDLL_CXX_FLAGS ${PHYSX_CXX_FLAGS})

SET(CMAKEDLL_CXX_FLAGS_DEBUG   ${PHYSXDLL_CXX_FLAGS_DEBUG})
SET(CMAKEDLL_CXX_FLAGS_CHECKED ${PHYSXDLL_CXX_FLAGS_CHECKED})
SET(CMAKEDLL_CXX_FLAGS_PROFILE ${PHYSXDLL_CXX_FLAGS_PROFILE})
SET(CMAKEDLL_CXX_FLAGS_RELEASE ${PHYSXDLL_CXX_FLAGS_RELEASE})

# set target src file variable
file(GLOB PHYSXDLL_SOURCES 
	../../src/*.cpp
)

# include target head file
include_directories(../../include)

add_library(PhysXDLL SHARED ${PHYSXDLL_SOURCES})

# Call into PhysX CMake scritps
ADD_SUBDIRECTORY(${PHYSX_ROOT_DIR}/compiler/public ${CMAKE_CURRENT_BINARY_DIR}/physx)

# Add physx libraries to target
target_link_libraries(PhysXDLL PUBLIC  	PhysXExtensions PhysX PhysXCooking	PhysXCommon	PhysXFoundation)