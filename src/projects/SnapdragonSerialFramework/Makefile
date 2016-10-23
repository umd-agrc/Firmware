# This is intended to be used as a PX4 app
# Thus, make should not be called on its own.
# These commands will be invoked appropriately from the PX4 make commands
all: snapdragon 

define sf_cmake_generate
mkdir -p build_$(1) && cd build_$(1) && cmake ../framework/$(1) -DOS=$(2) -DCMAKE_TOOLCHAIN_FILE=$(3) -DDF_ENABLE_TESTS=1
endef

.PHONY check_env:
	@if [ "${HEXAGON_SDK_ROOT}" = "" ]; then echo "HEXAGON_SDK_ROOT not set"; echo ${GETTING_STARTED_MSG}; false; fi
	@if [ "${HEXAGON_TOOLS_ROOT}" = "" ]; then echo "HEXAGON_TOOLS_ROOT not set"; echo ${GETTING_STARTED_MSG}; false; fi

snapdragon: qurt snapdragon_linaro

.PHONY qurt:
qurt:
	@$(call sf_cmake_generate,qurt,qurt,${PX4_FIRMWARE}/cmake/cmake_hexagon/toolchain/Toolchain-qurt.cmake)
	@cd build_qurt && $(MAKE)

.PHONY snapdragon_linaro:
snapdragon_linaro:
	@$(call sf_cmake_generate,snapdragon_linaro,linux,${PX4_FIRMWARE}/cmake/cmake_hexagon/toolchain/Toolchain-arm-linux-gnueabihf.cmake)
	@cd build_snapdragon_linaro && $(MAKE)

clean:
	@rm -rf build_*

load: qurt
	adb shell rm -f /usr/share/data/adsp/libuart_interface_skel.so /usr/share/data/adsp/libsf_serial_framework_qurt.so
	cd build_qurt && $(MAKE) libsf_serial_framework_qurt-load

