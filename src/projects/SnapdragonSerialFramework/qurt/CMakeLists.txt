#add_subdirectory(src)

px4_add_module(
	MODULE
  projects__SnapdragonSerialFramework__qurt
	SRCS
  #px4/px4_dummy.c
  src/SFUartDriver.cpp
  src/SFUartMain.cpp
	DEPENDS
		platforms__common
)

#target_link_libraries(
#	projects__SnapdragonSerialFramework__qurt
#	sf_serial_framework_qurt
#	uart_interface_skel	
#)
