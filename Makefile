cur_dir     := ${CURDIR}
dep_dir     := ./dependencies/
src_dir     := ./src/adptsysc
src_files   := $(wildcard $(src_dir)/*.cc) $(wildcard $(src_dir)/*.hh)
docker_dir  := ./scripts

PositiveWords := 1 true yes on ON TRUE YES

define normalize_onoff
$(if $(filter $(strip $1),$(PositiveWords)),ON,OFF)
endef

dbg ?= OFF
dbg := $(call normalize_onoff,$(dbg))

test-utils ?= OFF
test-utils := $(call normalize_onoff,$(test-utils))

test-dsplib ?= OFF
test-dsplib := $(call normalize_onoff,$(test-dsplib))

test-only ?= OFF
test-only := $(call normalize_onoff,$(test-only))

sysc-ams-en ?= ON
sysc-ams-en := $(call normalize_onoff,$(sysc-ams-en))

asan-en ?= OFF
asan-en := $(call normalize_onoff,$(asan-en))

tsan-en ?= OFF
tsan-en := $(call normalize_onoff,$(tsan-en))

njob ?= 1
NUM_CMAKE_JOBS ?= $(njob)

.PHONY: format cmake-format clean build build-test build-dsp-test run-test run-mem-sim run-r2sdffft-sim docker-run

build:
	cmake \
		-DADPT_TEST=OFF \
		-DADPT_DBUG=$(dbg) \
		-DADPT_USE_SYSTEMC_AMS=$(sysc-ams-en) \
		-DADPT_USE_ASAN=$(asan-en) \
		-DADPT_USE_TSAN=$(tsan-en) \
		-B ./build -S .
	cmake --build ./build --parallel $(NUM_CMAKE_JOBS)

build-test:
	cmake \
		-DADPT_TEST=ON \
		-DADPT_DBUG=$(dbg) \
		-DADPT_TEST_UTILS=$(test-utils) \
		-DADPT_TESTONLY=$(test-only) \
		-DADPT_TEST_DSPLIB=$(test-dsplib) \
		-DADPT_USE_ASAN=$(asan-en) \
		-DADPT_USE_TSAN=$(tsan-en) \
		-B ./build -S .
	cmake --build ./build --parallel $(NUM_CMAKE_JOBS)

build-dsp-test:
	cmake \
		-DADPT_TEST=ON \
		-DADPT_DBUG=$(dbg) \
		-DADPT_TEST_UTILS=OFF \
		-DADPT_TESTONLY=ON \
		-DADPT_TEST_DSPLIB=ON \
		-DADPT_USE_ASAN=OFF \
		-DADPT_USE_TSAN=OFF \
		-B ./build -S .
	cmake --build ./build --parallel $(NUM_CMAKE_JOBS)

run-mem-sim: build
	./build/bin/adptsysc --run-testbench -e syscmem --verbose \
		--mem-write-delay-cycles=1 --mem-read-delay-cycles=1

run-r2sdffft-sim: build
	./build/bin/adptsysc --run-testbench \
		-e r2sdf_fft_tlm \
		--verbose \
		--trace \
		--mem-write-delay-cycles=1 \
		--mem-read-delay-cycles=1
	@echo "Trace: $(cur_dir)/r2sdf_fft_dut_trace.log"

run-test: build-test build-dsp-test
	./build/bin/adptsysc-test

cmake-format:
	@echo "CMake format: ./CMakeLists.txt"
	cmake-format -i ./CMakeLists.txt

format:
	@echo "Format: $(src_files)"
	clang-format -i $(src_files)

docker-run:
	-@sh $(docker_dir)/docker-run.sh

clean:
	-@rm -rvf *.log ./build/ *.vcd *.hex