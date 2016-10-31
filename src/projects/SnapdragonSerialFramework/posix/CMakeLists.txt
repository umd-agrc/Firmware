#add_subdirectory(src)

# Use this directory as the PX4 module
if(COMMAND px4_add_module)
  px4_add_module(
    MODULE projects__SnapdragonSerialFramework__posix
    SRCS
    # px4/sf_dummy.cpp
      src/test_uart_posix.cpp
    DEPENDS
      platforms__common
  )

#  target_link_libraries(
    #    projects__SnapdragonSerialFramework__posix
    #    sf_serial_framework_posix
    #  )
endif()
