dep_dir	  	:= ./dependencies/
src_dir	  	:= ./src/adptsysc
src_files 	:= $(wildcard $(src_dir)/iirdesign-test.cc)
docker_dir	:= ./scripts

# Get specified feature set
PositiveWords = 1 true yes
define has_keyword
$(if $(filter $(firstword $(enable_$(strip $1))), $(PositiveWords)),1,0)
endef

# Enable GPU build 
dbg ?= OFF
ifeq ($(call has_keyword, dbg), 1)
override dbg := ON
endif

# Enable utils function test 
test-utils ?= OFF
ifeq ($(call has_keyword, test-utils), 1)
override test-utils := ON
endif

test-dsplib ?= OFF
ifeq ($(call has_keyword, test-dsplib), 1)
override test-dsplib := ON
endif

test-only ?= OFF
ifeq ($(call has_keyword, test-only), 1)
override test-only := ON
endif

# Enable cuda side utils function test 
sysc-ams-en ?= ON
ifeq ($(call has_keyword, sysc-ams-en), 1)
override sysc-ams-en := ON
endif

# Enable address sanitizer during compile
asan-en ?= OFF
ifeq ($(call has_keyword, asan-en), 1)
override asan-en := ON
endif

# Enable thread sanitizer during compile
tsan-en ?= OFF
ifeq ($(call has_keyword, tsan-en), 1)
override tsan-en := ON
endif

njob ?= 1
NUM_CMAKE_JOBS ?= $(njob)

.PHONY: format cmake_format clean build_test run_test

build:  # New target for building the main executable
	cmake \
		-DADPT_TEST=OFF \
		-DADPT_DBUG=$(dbg) \
		-DADPT_USE_SYSTEMC_AMS=$(sysc-ams-en) \
		-DADPT_USE_ASAN=$(asan-en) \
		-DADPT_USE_TSAN=$(tsan-en) \
		-B ./build -S .
	cmake --build ./build --parallel ${NUM_CMAKE_JOBS}

run: $(build)  # New target to run the main executable
	cmake --build ./build --parallel ${NUM_CMAKE_JOBS}
	./build/bin/adptsysc

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

run-test: $(build-test)
	cmake --build ./build --parallel ${NUM_CMAKE_JOBS}
	./build/bin/adptsysc-test

cmake-format:  
	@echo "Cmake Format: "./CMakeLists.txt
	cmake-format -i ./CMakeLists.txt

format:  
	@echo "Format: "$(src_files)  
	clang-format -i $(src_files) 
	
.PHONY: docker-run
docker-run:
	-@sh $(docker_dir)/docker-run.sh

.PHONY: clean
clean:
	-@rm -rvf *.log ./build/ *.vcd *.hex
