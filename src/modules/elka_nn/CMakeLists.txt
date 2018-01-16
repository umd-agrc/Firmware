px4_add_module(
	MODULE modules__elka_nn
	MAIN nn
	STACK_MAIN 4200
	STACK_MAX 3800
	COMPILE_FLAGS
	SRCS
    nn.cpp
    genann.cpp
    genann.h
	DEPENDS
		platforms__common
	)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
