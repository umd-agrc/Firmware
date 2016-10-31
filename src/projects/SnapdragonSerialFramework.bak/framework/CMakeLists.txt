include(../cmake/sf_common.cmake)

# Use this directory as the PX4 module
if(COMMAND px4_add_module)
  set(BUILD_DIR "${PX4_SOURCE_DIR}/build_${CONFIG}")
  set(QURT_BUILD_DIR "${BUILD_DIR}/build_qurt")
  set(QURT_LOAD_FILE "${QURT_BUILD_DIR}/qurt_make_dirs.txt")

  file(MAKE_DIRECTORY "${QURT_BUILD_DIR}/libs")
  file(APPEND "${QURT_LOAD_FILE}")

  # Step 1.
  set(SF_SERIAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
  execute_process(
    COMMAND make
    WORKING_DIRECTORY ${SF_SERIAL_DIR}
  )

  add_proj_load(
    TARGET_DIR ${SF_SERIAL_DIR}
    LOAD_FILE ${QURT_LOAD_FILE}
  )

  px4_add_module(
    MODULE projects__SnapdragonSerialFramework__framework
    SRCS
      px4/sf_dummy.cpp
    DEPENDS
      platforms__common
  )

	# Step 3.
  set(CMAKE_FIND_LIBRARY_SUFFIXES .a ${CMAKE_FIND_LIBRARY_SUFFIXES})

  #TODO only works with LINUX static libraries
  add_library(sf_serial_framework STATIC IMPORTED)
  set_property(TARGET sf_serial_framework PROPERTY IMPORTED_LOCATION
    "${SF_SERIAL_DIR}/build_snapdragon_linaro/libsf_serial_framework.a"
  )

  #find_library(libsf_serial_framework sf_serial_framework
  #  NAMES sf_serial_framework libsf_serial_framework libsf_serial_framework.a
  #  PATHS ${SF_SERIAL_DIR}/build_snapdragon_linaro
  #  NO_DEFAULT_PATH
  #)

  #find_library(test_out_fuck sf_serial_framework_qurt
  #  NAMES sf_serial_framework_qurt.so libsf_serial_framework_qurt.so
  #        sf_serial_framework_qurt libsf_serial_framework_qurt
  #  PATHS ${SF_SERIAL_DIR}/build_qurt
  #  NO_DEFAULT_PATH
  #)

  #if(NOT test_out_fuck)
  #  message("searched in ${SF_SERIAL_DIR}/build_qurt for sf lib")
  #  message(FATAL_ERROR "test_out_fuck library not found")
  #endif()

  #if(NOT libsf_serial_framework)
  #  message(FATAL_ERROR "sf_serial_framework library not found")
  #endif()

  #target_link_libraries(projects__SnapdragonSerialFramework__framework
  #   ${sf_serial_framework})
  target_link_libraries(projects__SnapdragonSerialFramework__framework
    sf_serial_framework)

  set(module_external_libraries
    ${module_external_libraries}
    sf_serial_framework
    CACHE INTERNAL "module_external_libraries"
  )
		
endif()

