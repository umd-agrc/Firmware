# Specify system to build for
SET(CMAKE_SYSTEM_NAME Generic)
# Specify processor to build for
set(CMAKE_SYSTEM_PROCESSOR arm)

SET(CMAKE_SYSTEM_VERSION 1)

# May need to use CmakeForceCompiler
# Specify cross compiler
#SET(CMAKE_FORCE_C_COMPILER /usr/bin/arm-none-eabi-gcc)
SET(CMAKE_FORCE_C_COMPILER arm-none-eabi-gcc GNU)
#SET(CMAKE_FORCE_CXX_COMPILER /usr/bin/arm-none-eabi-g++)
SET(CMAKE_FORCE_CXX_COMPILER arm-none-eabi-g++ GNU)

# Search for programs in the build host directories
set (CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# For libraries, headers, and packages in the target directories
set (CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set (CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set (CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
