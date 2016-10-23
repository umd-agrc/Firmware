include_directories(${HEXAGON_SDK_INCLUDES})
px4_add_module(
  MODULE projects__MyFirstProj
  # Name of main function
  MAIN MyFirstProj 
	STACK_MAIN 2000
	SRCS
    src/MyFirstProj.c
	DEPENDS
		platforms__common
)

# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
