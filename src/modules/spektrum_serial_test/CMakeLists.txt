px4_add_module(
	MODULE modules__spektrum_serial_test
	MAIN spektrum_serial_test
	STACK_MAIN 1300
	COMPILE_FLAGS
	SRCS
		spektrum_serial_test.cpp
		basic_uart.cpp
		basic_uart.h
	DEPENDS
		platforms__common
	)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
