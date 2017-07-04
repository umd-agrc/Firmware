px4_add_module(
	MODULE modules__elka__posix
	MAIN elka
	STACK_MAIN 1300
	COMPILE_FLAGS
	SRCS
		elka_posix.cpp
		elka_manager.cpp
		elka_devices.cpp
		inet_comm.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
