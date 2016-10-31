set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "$ENV{PX4_FIRMWARE}/cmake/cmake_hexagon")
include(linux_app)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g -std=gnu99 -fno-strict-aliasing -fdata-sections -fno-zero-initialized-in-bss -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-function -Wno-unused-variable -Wno-gnu-array-member-paren-init -Wno-cast-align -Wno-missing-braces -Wno-strict-aliasing") 

FASTRPC_STUB_GEN(../../common/uart_interface.idl)

LINUX_LIB(LIB_NAME sf_serial_framework_posix
  IDL_NAME uart_interface
  SOURCES SFMessenger.c
)
