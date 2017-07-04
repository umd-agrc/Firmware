px4_add_module(
	MODULE drivers__snapdragon_uart
	MAIN snapdragon_uart
	COMPILE_FLAGS
	SRCS
		snapdragon_uart.cpp
		snapdragon_uart_manager.cpp
		snapdragon_uart_devices.cpp
		basic_uart.cpp
	DEPENDS
		platforms__common
	)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
