if ($ENV{DSPAL_PATH} STREQUAL "")
message(FATAL_ERROR "Set DSPAL_PATH to <path>/<to>/<dspal>")
else()
set(DSPAL_PATH $ENV{DSPAL_PATH})
endif()

include(${CMAKE_TOOLCHAIN_FILE})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "$ENV{PX4_FIRMWARE}/cmake/cmake_hexagon")
include(qurt_lib)

include_directories(${DSPAL_PATH}/include)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g -std=gnu99 -fno-strict-aliasing -fdata-sections -fno-zero-initialized-in-bss -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-function -Wno-unused-variable -Wno-gnu-array-member-paren-init -Wno-cast-align -Wno-missing-braces -Wno-strict-aliasing") 

FASTRPC_STUB_GEN(../../common/uart_interface.idl)

QURT_LIB(LIB_NAME sf_serial_framework_qurt
	IDL_NAME uart_interface 
	SOURCES SFMessenger_qurt.c
)
