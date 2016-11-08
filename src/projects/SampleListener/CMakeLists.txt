px4_add_module(
	MODULE projects__SampleListener
  # Name of main function
	MAIN SampleListener 
	STACK_MAIN 2000
	SRCS
		SampleListener.cpp
	DEPENDS
		platforms__common
)

# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
