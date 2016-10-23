.PHONY all:
all: MyFirstProj MyFirstProj_adsp

.PHONY check env:
	@if [ "${HEXAGON_SDK_ROOT}" = "" ]; then echo "HEXAGON_SDK_ROOT not set"; echo ${GETTING_STARTED_MSG}; false; fi
	@if [ "${HEXAGON_TOOLS_ROOT}" = "" ]; then echo "HEXAGON_TOOLS_ROOT not set"; echo ${GETTING_STARTED_MSG}; false; fi

# Only build MyFirstProj for DSP
.PHONY MyFirstProj_adsp:
MyFirstProj_adsp: cmake_hexagon
	@mkdir -p build_adsp && cd build_adsp && cmake -Wno-dev ../adsp_proc -DCMAKE_TOOLCHAIN_FILE=$(PX4_FIRMWARE)/cmake/cmake_hexagon/toolchain/Toolchain-qurt.cmake
	@cd build_adsp && make

# Only build MyFirstProj for PX4
.PHONY MyFirstProj:
MyFirstProj: cmake_hexagon
	@cd $(PX4_FIRMWARE) && make posix_eagle_legacy_driver_default

.PHONY submodule:
submodule: cmake_hexagon

cmake_hexagon:
	echo "target cmake hexagon: FIXME later"

clean:
	@rm -rf build_adsp

load: MyFirstProj_adsp
	adb shell rm -f /usr/share/data/adsp/libdrone_cmds_skel.so /usr/share/data/adsp/libMyFirstProj.so /home/linaro/MyFirstProj*
	cd build_adsp && make MyFirstProj_adsp-load
