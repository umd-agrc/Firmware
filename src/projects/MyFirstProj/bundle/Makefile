############################################################################
# Copyright (c) 2015 Mark Charlebois. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name ATLFlight nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
############################################################################

all : gooddog-bundle
.PHONY : all

PHONIES = check_env gooddog-bundle submodule cmake_hexagon clean load-bundle
.PHONY : $(PHONIES)

INIT_MSG="This program is based off of ATL Flight HellowWorld program. For help:\n\
See https://github.com/ATLFlight/ATLFlightDocs/blob/master/GettingStarted.md"

check_env :
	@if [ "${HEXAGON_SDK_ROOT}" = "" ]; then echo "HEXAGON_SDK_ROOT not set"; echo ${INIT_MSG}; false; fi

# This target builds gooddog_app for apps proc, and libgooddog.so and libgooddog_skel.so for adsp proc
gooddog-bundle : check_env submodule
	@mkdir -p build_bundle && cd build_bundle && cmake -Wno-dev ../ -DCMAKE_TOOLCHAIN_FILE=${PX4_FIRMWARE}/cmake/cmake_hexagon/toolchain/Toolchain-qurt.cmake
	@cd build_bundle && make

submodule : cmake_hexagon

cmake_hexagon :
	git submodule update --init

clean :
	@rm -rf build_bundle

load-bundle : gooddog-bundle
	adb shell rm -f /usr/share/data/adsp/libexample_dog_skel.so /usr/share/data/adsp/libgooddog.so /home/linaro/gooddog*
	cd build_bundle && make gooddog-load
