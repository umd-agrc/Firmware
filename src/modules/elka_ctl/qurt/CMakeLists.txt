px4_add_module(
	MODULE modules__elka_ctl__qurt
	MAIN elka_ctl_qurt
	STACK_MAIN 600
	STACK_MAX 4200
	COMPILE_FLAGS
	SRCS
  #serial_defines.h
		elka_ctl_qurt.cpp
		qurt_utils.cpp
    #basic_uart.h
    #basic_uart.cpp
    #basic_navigator.h
    #basic_navigator.cpp
    #basic_estimator.h
    #basic_estimator.cpp
    #basic_control.h
    #basic_control.cpp
    #basic_comm.h
    #basic_comm.cpp
    #basic_messenger.h
    #basic_messenger.cpp
	DEPENDS
		platforms__common
	)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
