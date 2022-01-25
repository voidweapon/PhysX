pushd %~dp0
set ROOT_DIR=%CD%
popd


pushd %~dp0
SET ROOT_DIR=%ROOT_DIR:\=/%
SET PHYSX_ROOT_DIR=%ROOT_DIR%/../physx
SET PM_CMAKEMODULES_PATH=%ROOT_DIR%/../externals/CMakeModules
SET PM_PXSHARED_PATH=%ROOT_DIR%/../pxshared
SET PM_TARGA_PATH=%PHYSX_ROOT_DIR%/../externals/targa
SET PM_PATHS=%PM_CMAKEMODULES_PATH%;%PM_TARGA_PATH%
SET PM_AndroidNDK_PATH=%ROOT_DIR%/../externals/android-ndk-r10d

cmake -G "MinGW Makefiles" ^
      --no-warn-unused-cli ^
      -DCMAKE_PREFIX_PATH=%PM_PATHS% ^
      -DPHYSX_ROOT_DIR=%PHYSX_ROOT_DIR% ^
      -DPX_OUTPUT_LIB_DIR=%PHYSX_ROOT_DIR% ^
      -DPX_OUTPUT_BIN_DIR=%PHYSX_ROOT_DIR% ^
      -DTARGET_BUILD_PLATFORM=android ^
      -DCMAKE_MAKE_PROGRAM=%PM_AndroidNDK_PATH%/prebuilt/windows-x86_64/bin/make.exe ^
      -DCMAKE_TOOLCHAIN_FILE=%PM_CMAKEMODULES_PATH%/android/android.toolchain.cmake ^
      -DANDROID_NDK=%PM_AndroidNDK_PATH% ^
      -DANDROID_STL="gnustl_static" ^
      -DCM_ANDROID_FP="softfp" ^
      -DCMAKE_INSTALL_PREFIX="install/android-19/PhysX"^
      -DANDROID_NATIVE_API_LEVEL="android-19" ^
      -DANDROID_ABI="armeabi-v7a" ^
      -DCMAKE_BUILD_TYPE=Release 
popd     

pause
exit /b 0      