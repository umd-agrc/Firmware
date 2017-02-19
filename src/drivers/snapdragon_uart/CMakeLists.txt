px4_add_module(
	MODULE drivers__snapdragon_uart
	MAIN snapdragon_uart
	COMPILE_FLAGS
	SRCS
		snapdragon_uart.cpp
		snapdragon_uart_manager.cpp
		snapdragon_uart_devices.cpp
		basic_uart.c
	DEPENDS
		platforms__common
	)

# TODO link libraries correctly
# Can probably do this thru px4 CMake functions
# link_directories($ENV{MY_LIBS_PATH})
# target_link_libraries(drivers__snapdragon_uart ringbuf)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
