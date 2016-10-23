cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(version_major 0)
set(version_minor 1)
set(version_patch 0)
set(version "${version_major}.${version_minor}.${version_patch}")

set(package-contact "esolomon1221@gmail.com")

if ($ENV{PX4_FIRMWARE} STREQUAL "")
message(FATAL_ERROR "Set PX4_FIRMWARE to <path>/<to>/<px4 firmware>")
else()
set(PX4_FIRMWARE $ENV{PX4_FIRMWARE})
endif()

if ($ENV{DSPAL_PATH} STREQUAL "")
message(FATAL_ERROR "Set DSPAL_PATH to <path>/<to>/<dspal>")
else()
set(DSPAL_PATH $ENV{DSPAL_PATH})
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PX4_FIRMWARE}/cmake/cmake_hexagon")

if ("$ENV{ENABLE_TSAN}" STREQUAL "1")
message("Enabling Thread Sanitizer")

#warnings
set(warning_list
  -Wall
  -Werror
  -Wextra
  -Wno-sign-compare
  -Wshadow
  -Wfloat-equal
  -Wpointer-arith
  -Wmissing-declarations
  -Wno-unused-parameter
  -Werror=format-security
  -Werror=array-bounds
  -Wfatal-errors
  -Werror=unused-variable
  -Werror=reorder
  -Werror=uninitialized
  -Werror=init-self
  # cxx specific
  -Wno-missing-field-initializers
)

string (REPLACE ";" " " _TMP_STR "${warning_list}")
set(warning_flags ${_TMP_STR})

set(SF_CXX_FLAGS "-fsanitize=thread -std=c++11 ${warning_flags} -g" CACHE STRING "SF_CXX_FLAGS")

elseif("$ENV{ENABLE_ASAN}" STREQUAL "1")
message("Enabling Address Sanitizer")

# Used for unit testing, works with gcc 4.8 or clang
set(SF_CXX_FLAGS "-O1 -g -fsanitize=address -fno-omit-frame-pointer -std=c++11 ${warning_flags}" CACHE STRING "SF_CXX_FLAGS")

else()

  set(SF_CXX_FLAGS "-std=c++11 ${warning_flags} -g" CACHE STRING "SF_CXX_FLAGS")

endif()

# Enable this directory's flags:
set(CMAKE_CXX_FLAGS "${SF_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

# Enable CI flags from travis:
if ("$ENV{CI}")
  add_definitions(-DCI)
endif()

add_definitions(-D__STDC_FORMAT_MACROS)

set(sf_link_libs CACHE STRING "SF_LINK_LIBS")

# set SF_TARGET
include(cmake/sf_common.cmake)

#FIXME implement these targets/libraries if this project becomes useful
# outside of Snapdragon Flight with qurt
if (NOT "${SF_TARGET}" STREQUAL "qurt")

  list(APPEND sf_link_libs pthread)

  if (NOT APPLE)
    list(APPEND sf_link_libs rt)
  endif()
else()
  include_directories(${DSPAL_PATH}/include)
  add_subdirectory(os/qurt)
  list(APPEND sf_link_libs sf_qurt_stubs)

endif()

include_directories(
  framework/include
  #FIXME os/qurt/include does not exist
  os/qurt/include
  )

# Add framework library
add_subdirectory(framework/src)

if("${SF_ENABLE_TESTS}" STREQUAL "1")
  # Add unit test
  add_subdirectory(test)
endif()

message(STATUS "CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}")
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

# vim: set noet fenc=utf-8 ff=unix ft=cmake :
