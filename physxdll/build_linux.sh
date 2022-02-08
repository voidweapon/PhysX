#!/bin/bash +x

cd ./compiler/linux

export ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export PHYSX_ROOT_DIR="$ROOT_DIR/../../../physx"
export PM_PxShared_PATH="$PHYSX_ROOT_DIR/../pxshared"
export PM_CMakeModules_PATH="$PHYSX_ROOT_DIR/../externals/cmakemodules"
export PM_opengllinux_PATH="$PHYSX_ROOT_DIR/../externals/opengl-linux"
export PM_PATHS=$PM_opengllinux_PATH


cmake  -G "Unix Makefiles" \
      --no-warn-unused-cli \
      -DCMAKE_PREFIX_PATH=$PM_PATHS \
      -DPHYSX_ROOT_DIR=$PHYSX_ROOT_DIR \
      -DPX_OUTPUT_LIB_DIR=$PHYSX_ROOT_DIR \
      -DPX_OUTPUT_BIN_DIR=$PHYSX_ROOT_DIR \
      -DTARGET_BUILD_PLATFORM=linux \
      -DPX_OUTPUT_ARCH=x86 \
      -DCMAKE_C_COMPILER=clang \
      -DCMAKE_CXX_COMPILER=clang++ \
      -DCMAKE_BUILD_TYPE=Release \
