px4_add_module(
	MODULE modules__elka__common
	COMPILE_FLAGS
	SRCS
		elka.cpp
    elka_comm.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
