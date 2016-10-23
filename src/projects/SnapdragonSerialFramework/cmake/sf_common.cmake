# Copyright (C) 2015 Mark Charlebois. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in
# the documentation and/or other materials provided with the
# distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
# used to endorse or promote products derived from this software
# without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# Set SF_TARGET from OS and add any other platform detection logic

include (CMakeParseArguments)

if ("${SF_TARGET}" STREQUAL "")
  if ("${OS}" STREQUAL "")
    message(FATAL_ERROR "OS not defined")
  endif()

  if("${OS}" STREQUAL "posix")
    if (APPLE)
      set(SF_TARGET darwin)

      if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8.0)
        message(FATAL_ERROR "PX4 Firmware requires XCode 8 or newer on Mac OS. Version installed on this system: ${CMAKE_CXX_COMPILER_VERSION}")
      endif()

      EXEC_PROGRAM(uname ARGS -v  OUTPUT_VARIABLE DARWIN_VERSION)
      STRING(REGEX MATCH "[0-9]+" DARWIN_VERSION ${DARWIN_VERSION})
      # message(STATUS "DF Darwin Version: ${DARWIN_VERSION}")
      if (DARWIN_VERSION LESS 16)
        add_definitions(
          -DCLOCK_MONOTONIC=1
          -D__DF_APPLE_LEGACY
          )
      endif()
    else()
		#	if(SNAPDRAGON STREQUAL "1")
		#		set(SF_TARGET qurt)
		#	else()
		#		set(SF_TARGET linux)
		#	endif()
			set(SF_TARGET linux)
    endif()
  else()
    set(SF_TARGET ${OS})
  endif()
endif()

function (sf_add_library sf_library_name)
  set(args "${ARGN}")
  add_library (${sf_library_name} ${args})

  if ("${SF_TARGET}" STREQUAL "qurt")
    set_property(TARGET ${sf_library_name} PROPERTY POSITION_INDEPENDENT_CODE TRUE)
  endif()
endfunction ()

function (add_proj_load)
	set(options)
  set(oneValueArgs TARGET_DIR LOAD_FILE)
  set(multiValueArgs)
	cmake_parse_arguments(add_proj_load "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )
  file(READ ${add_proj_load_LOAD_FILE} load_contents)
  if(NOT "${load_contents}" MATCHES "(\n|^)${add_proj_load_TARGET_DIR}(\n|$)")
    #Append to the file
    message("appending ${add_proj_load_TARGET_DIR} to ${add_proj_load_LOAD_FILE}")
    file(APPEND ${add_proj_load_LOAD_FILE} "${add_proj_load_TARGET_DIR}\n")
  endif()
endfunction ()
