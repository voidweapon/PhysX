
IF(TARGET_BUILD_PLATFORM STREQUAL "mac" OR TARGET_BUILD_PLATFORM STREQUAL "ios")
	cmake_minimum_required(VERSION 3.12)
ELSE()
	cmake_minimum_required(VERSION 3.11)
ENDIF()


# This is required to be defined by external callers!
IF(NOT DEFINED PHYSX_ROOT_DIR)
	MESSAGE(FATAL_ERROR "PHYSX_ROOT_DIR variable wasn't set.")
ENDIF()

IF(NOT EXISTS ${PHYSX_ROOT_DIR})
	MESSAGE(FATAL_ERROR "PHYSX_ROOT_DIR variable was invalid.")
ENDIF()

PROJECT(PhysXDLL C CXX)


STRING(TOLOWER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_LOWERCASE)

SET(PHYSX_WARNING_DISABLES "-Wno-invalid-offsetof -Wno-maybe-uninitialized  -Wno-unused-variable -Wno-variadic-macros -Wno-array-bounds")

IF(${ANDROID_ABI} STREQUAL "armeabi-v7a")
	SET(PHYSX_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -Wno-unknown-pragmas -Wextra -Wpedantic -std=c++11  -fno-exceptions -ffunction-sections -fdata-sections -funwind-tables -fomit-frame-pointer -funswitch-loops -finline-limit=300 -fno-strict-aliasing -fstack-protector ${PHYSX_WARNING_DISABLES}" CACHE INTERAL "PhysXDLL CXX")
ELSEIF(${ANDROID_ABI} STREQUAL "armeabi-v7a with NEON")
	SET(PHYSX_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -Wno-unknown-pragmas -Wextra -Wpedantic -std=c++11  -fno-exceptions -ffunction-sections -funwind-tables -fomit-frame-pointer -funswitch-loops -finline-limit=300 -fno-strict-aliasing -fstack-protector ${PHYSX_WARNING_DISABLES}" CACHE INTERAL "PhysXDLL CXX")
ELSEIF(${ANDROID_ABI} STREQUAL "arm64-v8a")
	SET(PHYSX_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -Wno-unknown-pragmas -Wextra -Wpedantic -std=c++11  -fno-exceptions -ffunction-sections -fdata-sections ${PHYSX_WARNING_DISABLES} " CACHE INTERAL "PhysXDLL CXX")
ELSEIF(${ANDROID_ABI} STREQUAL "x86")
	SET(PHYSX_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -Wno-unknown-pragmas -Wextra -Wpedantic -std=c++11  -fno-exceptions -ffunction-sections -fdata-sections ${PHYSX_WARNING_DISABLES} -fpack-struct=8 -malign-double " CACHE INTERAL "PhysXDLL CXX")
ELSEIF(${ANDROID_ABI} STREQUAL "x86_64")
	SET(PHYSX_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -Wno-unknown-pragmas -Wextra -Wpedantic -std=c++11  -fno-exceptions -ffunction-sections -fdata-sections ${PHYSX_WARNING_DISABLES} -mstackrealign -msse3 " CACHE INTERAL "PhysXDLL CXX")
ENDIF()

# Build debug info for all configurations
SET(PHYSXDLL_CXX_FLAGS_RELEASE "-O3 -g -DNDEBUG -D__STDC_LIMIT_MACROS" CACHE INTERAL "PhysX Release CXX Flags")

# These flags are local to the directory the CMakeLists.txt is in
SET(CMAKE_CXX_FLAGS ${PHYSX_CXX_FLAGS})

SET(CMAKE_CXX_FLAGS_RELEASE ${PHYSXDLL_CXX_FLAGS_RELEASE})

# Controls PX_NVTX for all projects
SET(NVTX_FLAG "PX_NVTX=0")

# Disable cuda and dx for all projects on Android
SET(PHYSXDLL_ANDROID_RELEASE_COMPILE_DEFS "NDEBUG;PHYSXDLL_EXPORTS;PX_SUPPORT_PVD=0" CACHE INTERNAL "Release PhysX preprocessor definitions")


# set target src file variable
file(GLOB PHYSXDLL_SOURCES 
	src/*.cpp
)

# include target head file
include_directories(include)

add_library(PhysXDLL SHARED ${PHYSXDLL_SOURCES})

# Call into PhysX CMake scritps
ADD_SUBDIRECTORY(${PHYSX_ROOT_DIR}/compiler/public ${CMAKE_CURRENT_BINARY_DIR}/physx)

# Add physx libraries to target
target_link_libraries(PhysXDLL PUBLIC  	PhysXExtensions PhysX	PhysXPvdSDK	PhysXVehicle	PhysXCharacterKinematic	PhysXCooking	PhysXCommon	PhysXFoundation)