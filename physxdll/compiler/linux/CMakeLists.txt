cmake_minimum_required(VERSION 3.12)

# This is required to be defined by external callers!
IF(NOT DEFINED PHYSX_ROOT_DIR)
	MESSAGE(FATAL_ERROR "PHYSX_ROOT_DIR variable wasn't set.")
ENDIF()

IF(NOT EXISTS ${PHYSX_ROOT_DIR})
	MESSAGE(FATAL_ERROR "PHYSX_ROOT_DIR variable was invalid.")
ENDIF()

STRING(TOLOWER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_LOWERCASE)

PROJECT(PhysXDLL C CXX)

#TODO: Fix warnings
SET(CLANG_WARNINGS "-ferror-limit=0 -Wall -Wextra  -Wstrict-aliasing=2 -Weverything -Wno-documentation-deprecated-sync -Wno-documentation-unknown-command -Wno-gnu-anonymous-struct -Wno-undef -Wno-unused-function -Wno-nested-anon-types -Wno-float-equal -Wno-padded -Wno-weak-vtables -Wno-cast-align -Wno-conversion -Wno-missing-noreturn -Wno-missing-variable-declarations -Wno-shift-sign-overflow -Wno-covered-switch-default -Wno-exit-time-destructors -Wno-global-constructors -Wno-missing-prototypes -Wno-unreachable-code -Wno-unused-macros -Wno-unused-member-function -Wno-used-but-marked-unused -Wno-weak-template-vtables -Wno-deprecated -Wno-non-virtual-dtor -Wno-invalid-noreturn -Wno-return-type-c-linkage -Wno-reserved-id-macro -Wno-c++98-compat-pedantic -Wno-unused-local-typedef -Wno-old-style-cast -Wno-newline-eof -Wno-unused-private-field -Wno-format-nonliteral -Wno-implicit-fallthrough -Wno-undefined-reinterpret-cast -Wno-disabled-macro-expansion -Wno-zero-as-null-pointer-constant -Wno-shadow -Wno-unknown-warning-option")
SET(GCC_WARNINGS "-Wall  -Wno-invalid-offsetof -Wno-uninitialized")

IF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	# using Clang	  
	SET(PHYSXDLL_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fno-rtti -fno-exceptions -ffunction-sections -fdata-sections -fstrict-aliasing ${CLANG_WARNINGS}" CACHE INTERAL "PhysX CXX")
ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	SET(PHYSXDLL_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fno-rtti -fno-exceptions -ffunction-sections -fdata-sections -fno-strict-aliasing ${GCC_WARNINGS}" CACHE INTERAL "PhysX CXX")
ENDIF()

# Build debug info for all configurations
SET(PHYSXDLL_CXX_FLAGS_DEBUG   "-O0 -g3 -gdwarf-2" CACHE INTERNAL "PhysXDll Debug CXX Flags")
SET(PHYSXDLL_CXX_FLAGS_CHECKED "-O3 -g3 -gdwarf-2" CACHE INTERNAL "PhysXDll Checked CXX Flags")
SET(PHYSXDLL_CXX_FLAGS_PROFILE "-O3" CACHE INTERNAL "PhysXDll Profile CXX Flags")
SET(PHYSXDLL_CXX_FLAGS_RELEASE "-O3" CACHE INTERNAL "PhysXDll Release CXX Flags")

#This will properly propagate the linker flags for all build types (e.g., Debug, Release, etc...) to look for .so files in the current working directory first.
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath='$ORIGIN'")

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

