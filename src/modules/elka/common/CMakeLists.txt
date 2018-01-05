if(${OS} STREQUAL "posix" OR ${OS} STREQUAL "qurt")

px4_add_module(
	MODULE modules__elka__common
	COMPILE_FLAGS
	SRCS
		elka.cpp
    elka_comm.cpp
	DEPENDS
		platforms__common
)

elseif(${OS} STREQUAL "FreeRTOS")

add_library(elka_comm STATIC
	elka.cpp
	elka_comm.cpp
)

set(ELKA_LINK_LIBS
  "${ELKA_LINK_LIBS} elka_comm"
)

else()

message(
	FATAL_ERROR
	"OS ${OS} not supported with elka_comm library"
)

endif()
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
