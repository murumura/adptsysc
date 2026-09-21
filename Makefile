cur_dir    := $(CURDIR)
dep_dir    := ./dependencies
src_dir    := ./src/adptsysc
src_files  := $(wildcard $(src_dir)/*.cc) $(wildcard $(src_dir)/*.hh)
docker_dir := ./scripts

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

FFT_FX_TOL ?= 1e-2
WBCIC_FX_TOL ?= 1e-2
WBCIC_OUT ?= $(cur_dir)/build/wbcic_testvectors

.PHONY: \
	build \
	build-test \
	build-dsp-test \
	run-test \
	run-dsp-test \
	run-mem-sim \
	run-mem-hex \
	run-mem-bin \
	run-mem-raw \
	run-mem-all \
	run-r2sdffft-sim \
	run-r2sdffft-fx \
	run-r2sdffft-ifft \
	run-r2sdffft-ifft-fx \
	run-r2sdffft-all \
	run-mem-cycle \
	run-r2sdffft-cycle \
	run-r2sdffft-cycle-ifft \
	run-fdaf-cycle \
	run-cycle-all \
	build-wbcic-engine \
	run-wbcic-engine \
	run-wbcic-engine-fx \
	cmake-format \
	format \
	docker-run \
	clean

# ============================================================================================
# Build
# ============================================================================================

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
		-DADPT_USE_SYSTEMC_AMS=$(sysc-ams-en) \
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
		-DADPT_USE_SYSTEMC_AMS=$(sysc-ams-en) \
		-DADPT_USE_ASAN=OFF \
		-DADPT_USE_TSAN=OFF \
		-B ./build -S .
	cmake --build ./build --parallel $(NUM_CMAKE_JOBS)

run-test: build-test
	./build/bin/adptsysc-test

run-dsp-test: build-dsp-test
	./build/bin/adptsysc-test

# ============================================================================================
# SyscMemory TLM tests
# ============================================================================================

run-mem-sim: build
	./build/bin/adptsysc \
		--run-testbench \
		-e syscmem \
		--verbose \
		--mem-write-delay-cycles=1 \
		--mem-read-delay-cycles=1

run-mem-hex: build
	./build/bin/adptsysc \
		--run-testbench \
		-e syscmem \
		--verbose \
		--text-output=$(cur_dir)/fixed_dump_hex.mem \
		--oformat=hex \
		--mem-write-delay-cycles=1 \
		--mem-read-delay-cycles=1
	@echo "---- fixed_dump_hex.mem head ----"
	@head -25 $(cur_dir)/fixed_dump_hex.mem
	@echo "---- fixed_dump_hex.mem tail ----"
	@tail -5 $(cur_dir)/fixed_dump_hex.mem

run-mem-bin: build
	./build/bin/adptsysc \
		--run-testbench \
		-e syscmem \
		--verbose \
		--text-output=$(cur_dir)/fixed_dump_bin.mem \
		--oformat=binary \
		--mem-write-delay-cycles=1 \
		--mem-read-delay-cycles=1
	@echo "---- fixed_dump_bin.mem head ----"
	@head -10 $(cur_dir)/fixed_dump_bin.mem

run-mem-raw: build
	./build/bin/adptsysc \
		--run-testbench \
		-e syscmem \
		--verbose \
		--output=$(cur_dir)/raw_dump.bin \
		--mem-write-delay-cycles=1 \
		--mem-read-delay-cycles=1
	@echo "---- raw_dump.bin ----"
	@ls -lh $(cur_dir)/raw_dump.bin

run-mem-all: \
	run-mem-sim \
	run-mem-hex \
	run-mem-bin \
	run-mem-raw

# ============================================================================================
# R2SDF FFT TLM tests
# ============================================================================================

run-r2sdffft-sim: build
	./build/bin/adptsysc \
		--run-testbench \
		-e r2sdf_fft_tlm \
		--verbose \
		--trace \
		--mem-write-delay-cycles=1 \
		--mem-read-delay-cycles=1

run-r2sdffft-fx: build
	./build/bin/adptsysc \
		--run-testbench \
		-e r2sdf_fft_tlm \
		--verbose \
		--fixedpoint-eval \
		--fixedpoint-tol=$(FFT_FX_TOL) \
		--mem-write-delay-cycles=1 \
		--mem-read-delay-cycles=1

run-r2sdffft-ifft: build
	./build/bin/adptsysc \
		--run-testbench \
		-e r2sdf_fft_tlm \
		--ifft \
		--verbose \
		--mem-write-delay-cycles=1 \
		--mem-read-delay-cycles=1

run-r2sdffft-ifft-fx: build
	./build/bin/adptsysc \
		--run-testbench \
		-e r2sdf_fft_tlm \
		--ifft \
		--verbose \
		--fixedpoint-eval \
		--fixedpoint-tol=$(FFT_FX_TOL) \
		--mem-write-delay-cycles=1 \
		--mem-read-delay-cycles=1

run-r2sdffft-all: \
	run-r2sdffft-sim \
	run-r2sdffft-fx \
	run-r2sdffft-ifft \
	run-r2sdffft-ifft-fx

# ============================================================================================
# Cycle-accurate SystemC tests
# ============================================================================================

run-mem-cycle: build
	./build/bin/adptsysc \
		--run-testbench \
		-e syscmem_cycle \
		--verbose

run-r2sdffft-cycle: build
	./build/bin/adptsysc \
		--run-testbench \
		-e r2sdf_fft_cycle \
		--verbose

run-r2sdffft-cycle-ifft: build
	./build/bin/adptsysc \
		--run-testbench \
		-e r2sdf_fft_cycle \
		--ifft \
		--verbose

run-fdaf-cycle: build
	./build/bin/adptsysc \
		--run-testbench \
		-e overlap_save_fdaf_cycle \
		--verbose

run-cycle-all: \
	run-mem-cycle \
	run-r2sdffft-cycle \
	run-r2sdffft-cycle-ifft \
	run-fdaf-cycle

# ============================================================================================
# WBCIC Fig. 3(b): engine testbench via the normal multi-architecture build
# ============================================================================================

build-wbcic-engine:
	cmake \
		-DADPT_TEST=OFF \
		-DADPT_TESTONLY=OFF \
		-DADPT_DBUG=$(dbg) \
		-DADPT_USE_SYSTEMC_AMS=OFF \
		-DADPT_USE_ASAN=$(asan-en) \
		-DADPT_USE_TSAN=$(tsan-en) \
		-B ./build -S .
	cmake --build ./build --parallel $(NUM_CMAKE_JOBS)

run-wbcic-engine: build-wbcic-engine
	./build/bin/adptsysc \
		--run-testbench \
		-e wideband_cic_engine \
		--signal-trace \
		--sv-trace \
		--sv-trace-dir=$(WBCIC_OUT)/sv \
		--text-output=$(WBCIC_OUT)/output_directory_only \
		--verbose

run-wbcic-engine-fx: build-wbcic-engine
	./build/bin/adptsysc \
		--run-testbench \
		-e wideband_cic_engine \
		--fixedpoint-eval \
		--fixedpoint-tol=$(WBCIC_FX_TOL) \
		--signal-trace \
		--sv-trace \
		--sv-trace-dir=$(WBCIC_OUT)/sv \
		--text-output=$(WBCIC_OUT)/output_directory_only \
		--verbose

# ============================================================================================
# Formatting / Docker / cleanup
# ============================================================================================

cmake-format:
	@echo "CMake format: ./CMakeLists.txt"
	cmake-format -i ./CMakeLists.txt

format:
	@echo "Format: $(src_files)"
	clang-format -i $(src_files)

docker-run:
	-@sh $(docker_dir)/docker-run.sh

clean:
	-@rm -rvf \
		*.log \
		*.vcd \
		*.hex \
		*.mem \
		*.bin \
		svtrace \
		./build/