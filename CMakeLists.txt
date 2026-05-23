cmake_minimum_required(VERSION 3.27 FATAL_ERROR)
cmake_policy(SET CMP0167 NEW)

project(
  adptsysc
  VERSION 0.1.0
  LANGUAGES CXX
  DESCRIPTION "A C++20/SystemC for adative filter modeling"
  HOMEPAGE_URL "https://github.com/murumura/AdptFilter_SysC.git"
)

if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
  message(FATAL_ERROR "In-source builds are not allowed. Use a separate build directory.")
endif()

# ============================================================================================
# Options
# ============================================================================================
option(DOWNLOAD_EIGEN "Download EIGEN (requires CMake 3.11+)" OFF)
option(ADPT_TESTONLY "Build unit tests for adptsysc only" OFF)
option(ADPT_TEST "Build unit tests for adptsysc" OFF)
option(ADPT_DBUG "Enable Debugging Mode" OFF)
option(ADPT_VERBOSE "Display build information" ON)
option(ADPT_TEST_DSPLIB "Enable test signal processing functions" OFF)
option(ADPT_TEST_UTILS "Enable test utils functions" OFF)
option(ADPT_USE_SYSTEMC_AMS "Enable SystemC AMS support" ON)
option(ADPT_USE_ASAN "Enable AddressSanitizer" OFF)
option(ADPT_USE_TSAN "Enable ThreadSanitizer" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# ============================================================================================
# Debug/Verbose settings
# ============================================================================================
set(ADPT_DEFS "")
if(ADPT_DBUG)
  message(STATUS "--[ADPT-INFO] Debug Mode : ON (via ADPT_DBUG)")
  list(APPEND ADPT_DEFS -DDBUG_MODE)
endif()

message(STATUS "--[ADPT-INFO] Unit tests only: ${ADPT_TESTONLY}")
message(STATUS "--[ADPT-INFO] Unit tests : ${ADPT_TEST}")
message(STATUS "--[ADPT-INFO] ADPT-TEST-DSPLIB : ${ADPT_TEST_DSPLIB}")
message(STATUS "--[ADPT-INFO] Verbose output: ${ADPT_VERBOSE}")
message(STATUS "--[ADPT-INFO] AddressSanitizer: ${ADPT_USE_ASAN}")
message(STATUS "--[ADPT-INFO] ThreadSanitizer: ${ADPT_USE_TSAN}")


if("${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
endif()

if (ADPT_TESTONLY AND NOT ADPT_TEST)
  message(
    FATAL_ERROR
      "\n--[ADPT-ERROR] Test-only mode requires the test option to be enabled!\n"
      "Please configure with:\n"
      "  -DADPT_TEST=ON\n"
      "Current ADPT_TEST=${ADPT_TEST}\n"
      "Current ADPT_TESTONLY=${ADPT_TESTONLY}\n"
  )
endif()

# ============================================================================================
# Dependencies: Eigen, GoogleTest
# ============================================================================================
include(FetchContent)

FetchContent_Declare(json
  GIT_REPOSITORY https://github.com/nlohmann/json
  GIT_TAG v3.12.0
)
FetchContent_MakeAvailable(json)

FetchContent_Declare(
  Eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG master
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  EXCLUDE_FROM_ALL
)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(Eigen)

if(NOT eigen_POPULATED)
  if(CMAKE_VERSION VERSION_LESS 3.27.0)
    FetchContent_Populate(Eigen)
    add_subdirectory(${eigen_SOURCE_DIR} ${eigen_BINARY_DIR} EXCLUDE_FROM_ALL)
  else()
    FetchContent_MakeAvailable(Eigen)
  endif()
endif()

message(STATUS "--[ADPT-INFO] Success. Populated Eigen3 at ${eigen_SOURCE_DIR}")
message(STATUS "--[ADPT-INFO] Eigen include path: ${eigen_SOURCE_DIR}")

if(ADPT_TEST)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
  )
  FetchContent_MakeAvailable(googletest)
  find_package(GTest REQUIRED)
endif()

# ============================================================================================
# Find SystemC
# ============================================================================================

if(NOT DEFINED SystemC_DIR AND DEFINED ENV{SYSTEMC_HOME})
  set(SystemC_DIR "$ENV{SYSTEMC_HOME}/lib/cmake/SystemC")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(SystemC REQUIRED)

if(NOT SystemC_FOUND)
  message(FATAL_ERROR "--[ADPT-ERROR] SystemC not found. Set SYSTEMC_HOME or install SystemC properly.")
else()
  message(STATUS "--[ADPT-INFO] SystemC include: ${SystemC_INCLUDE_DIRS}")
  message(STATUS "--[ADPT-INFO] SystemC libs   : ${SystemC_LIBRARIES}")
endif()

# ============================================================================================
# Find SystemC-AMS
# ============================================================================================
if(ADPT_USE_SYSTEMC_AMS)
  if(NOT DEFINED SystemC_DIR AND DEFINED ENV{SYSTEMC_HOME})
    set(SystemC_DIR "$ENV{SYSTEMC_HOME}/lib/cmake/SystemC")
  endif()

  list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
  find_package(SystemCAMS REQUIRED)
  
  if(NOT SystemCAMS_FOUND)
    message(FATAL_ERROR "--[ADPT-ERROR] SystemC-AMS not found. Set SYSTEMC_AMS_HOME or install SystemC-AMS properly.")
  else()
    message(STATUS "--[ADPT-INFO] SystemC-AMS include: ${SystemCAMS_INCLUDE_DIRS}")
    message(STATUS "--[ADPT-INFO] SystemC-AMS libs   : ${SystemCAMS_LIBRARIES}")
  endif()
endif()

# ============================================================================================
# Paths
# ============================================================================================
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

# ============================================================================================
# Source Files & Template Files
# ============================================================================================
# Almost all functions are template in adpt which take a target type
# (e.g. LMSArch) as its type parameter. Since we suport more than 10
# targets, compiling a single source file for all the targets is very
# slow.
#
# As a workaround, we create a .cc file for each target and spawn many
# compiler instances. This is hacky but greatly reduces compile time
# on a multicore machine.
list(APPEND ADPT_TARGETS
  SyscMemArch
  R2SdfFFTTLMArch
)

list(GET ADPT_TARGETS 0 ADPT_FIRST_TARGET)
message(STATUS "--[DEBUG] Setting ADPT_FIRST_TARGET to: ${ADPT_FIRST_TARGET}")


set(ADPT_DSP_SOURCES
  src/adptsysc/dsplib.hh
  src/adptsysc/dsplib.cc
  src/adptsysc/armodel.cc
  src/adptsysc/fir-design.cc
  src/adptsysc/iir-design.cc
  src/adptsysc/design-lib.hh
  src/adptsysc/adpt-optimizer.hh
  src/adptsysc/adpt-optimizer.cc
  src/adptsysc/adpt-filter.hh
)

set(ADPT_SOURCES
  src/adptsysc/adptsysc-entry.cc
  src/adptsysc/common.hh
  src/adptsysc/arch.hh
  src/adptsysc/adptsysc.hh
  src/adptsysc/sysc-mem.hh
  src/adptsysc/object.hh
  src/adptsysc/lib.cc
  src/adptsysc/sysc-cmplxmul.hh
  src/adptsysc/sysc-shiftreg.hh
  src/adptsysc/sysc-r2sdffft.hh
  src/adptsysc/syscfx-utils.cc
)

set(ADPT_SOURCES_TEST 
  src/adptsysc/adptsysc-test.cc
)

if(ADPT_TEST_DSPLIB)
  list(APPEND ADPT_SOURCES_TEST 
    src/adptsysc/dsplib-test.cc
    src/adptsysc/iirdesign-test.cc
    src/adptsysc/firdesign-test.cc
    src/adptsysc/design-lib-test.cc
    src/adptsysc/adpt-filter-test.cc
  )
endif()

if(ADPT_TEST_UTILS)
  list(APPEND ADPT_SOURCES_TEST 
    src/adptsysc/sysc-utils-test.cc
  )
endif()

set(ADPT_GENERATED_FILES "")

function(adpt_instantiate_templates SOURCE TARGET)
  set(options)
  set(oneValueArgs FEATURE_DEFINE)
  cmake_parse_arguments(AIT "${options}" "${oneValueArgs}" "" ${ARGN})

  set(OUTFILE ${CMAKE_BINARY_DIR}/${SOURCE}.${TARGET}.cc)
  message(STATUS "--[DEBUG] Instantiate ${SOURCE} for target ${TARGET}")

  set(GEN_TEXT "#define ADPT_TARGET ${TARGET}\n")
  if(AIT_FEATURE_DEFINE)
    string(APPEND GEN_TEXT "#define ${AIT_FEATURE_DEFINE} 1\n")
  endif()
  string(APPEND GEN_TEXT "#include \"${CMAKE_SOURCE_DIR}/${SOURCE}\"\n")

  file(WRITE ${OUTFILE} "${GEN_TEXT}")

  list(APPEND ADPT_GENERATED_FILES ${OUTFILE})
  set(ADPT_GENERATED_FILES "${ADPT_GENERATED_FILES}" PARENT_SCOPE)
endfunction()

function(adpt_register_template_source SOURCE)
  set(options)
  set(oneValueArgs FEATURE_DEFINE)
  set(multiValueArgs TARGETS)
  cmake_parse_arguments(ARTS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(NOT ARTS_TARGETS)
    message(FATAL_ERROR "adpt_register_template_source(${SOURCE}) requires TARGETS")
  endif()

  foreach(TGT IN LISTS ARTS_TARGETS)
    if(ARTS_FEATURE_DEFINE)
      adpt_instantiate_templates(${SOURCE} ${TGT} FEATURE_DEFINE ${ARTS_FEATURE_DEFINE})
    else()
      adpt_instantiate_templates(${SOURCE} ${TGT})
    endif()
    set(ADPT_GENERATED_FILES "${ADPT_GENERATED_FILES}" PARENT_SCOPE)
  endforeach()
endfunction()

if (NOT ADPT_TESTONLY)
  foreach(TARGET IN LISTS ADPT_TARGETS)
    set(HAVE_${TARGET} 1)
  endforeach()

  adpt_register_template_source(
    src/adptsysc/cmdline.cc
    TARGETS ${ADPT_TARGETS}
  )

  adpt_register_template_source(
    src/adptsysc/common.cc
    TARGETS ${ADPT_TARGETS}
  )

  adpt_register_template_source(
    src/adptsysc/adptsysc-main.cc
    TARGETS ${ADPT_TARGETS}
  )

  adpt_register_template_source(
    src/adptsysc/sysc-mem.cc
    TARGETS SyscMemArch R2SdfFFTTLMArch
    FEATURE_DEFINE ADPT_ENABLE_SYSC_MEM
  )

  adpt_register_template_source(
    src/adptsysc/sysc-r2sdffft.cc
    TARGETS R2SdfFFTTLMArch
    FEATURE_DEFINE ADPT_ENABLE_R2SDF
  )

  configure_file(src/adptsysc/config.h.in ${CMAKE_SOURCE_DIR}/src/adptsysc/config.hh)
  list(APPEND ADPT_GENERATED_FILES ${CMAKE_SOURCE_DIR}/src/adptsysc/config.hh)
endif()

# ============================================================================================
# Runtime Python plotting script
# ============================================================================================

set(ADPT_PLOT_SCRIPT
  ${CMAKE_SOURCE_DIR}/src/adptsysc/adptplot.py
)

set(ADPT_PLOT_SCRIPT_DST
  ${CMAKE_BINARY_DIR}/adptplot.py
)

add_custom_command(
  OUTPUT ${ADPT_PLOT_SCRIPT_DST}
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${ADPT_PLOT_SCRIPT}
          ${ADPT_PLOT_SCRIPT_DST}
  DEPENDS ${ADPT_PLOT_SCRIPT}
  COMMENT "--[ADPT] Copy adptplot.py to build directory"
)

add_custom_target(adpt_plot_script ALL
  DEPENDS ${ADPT_PLOT_SCRIPT_DST}
)

# ============================================================================================
# Target: adptsysc_lib
# ============================================================================================
# Define the library once
add_library(adptsysc_lib STATIC)
# treat Eigen as a system library, suppressing its warnings
target_include_directories(adptsysc_lib    
  PUBLIC 
    ${eigen_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)

# Add hand-written source files
target_sources(adptsysc_lib PUBLIC ${ADPT_DSP_SOURCES})

if (NOT ADPT_TESTONLY) 
  # Add generated template files
  target_sources(adptsysc_lib PRIVATE ${ADPT_GENERATED_FILES})

  # Add hand-written source files
  target_sources(adptsysc_lib PRIVATE ${ADPT_SOURCES})

  message(STATUS "  ${ADPT_GENERATED_FILES}")
  target_include_directories(adptsysc_lib
    PUBLIC
      ${SystemC_INCLUDE_DIRS}
      ${SystemCAMS_INCLUDE_DIRS}   
  )
endif()

target_compile_definitions(adptsysc_lib PUBLIC ${ADPT_DEFS})
target_compile_features(adptsysc_lib PUBLIC cxx_std_20)
target_link_libraries(adptsysc_lib PUBLIC nlohmann_json::nlohmann_json)
if (NOT ADPT_TESTONLY) 
  target_link_libraries(adptsysc_lib
    PUBLIC
      ${SystemC_LIBRARIES}
      ${SystemCAMS_LIBRARIES}
  )
endif()

# ============================================================================================
# Executable: adptsysc
# ============================================================================================
if (NOT ADPT_TESTONLY) 
  add_executable(adptsysc src/adptsysc/adptsysc-entry.cc)
  target_link_libraries(adptsysc PRIVATE adptsysc_lib)
  if(ADPT_USE_TSAN)
    target_link_options(adptsysc PRIVATE -fsanitize=thread)
    target_compile_options(adptsysc PRIVATE -fsanitize=thread)
  endif()

  if(ADPT_USE_ASAN)
    target_link_options(adptsysc PRIVATE -fsanitize=address -fsanitize=undefined)
    target_compile_options(adptsysc PRIVATE -fsanitize=address -fsanitize=undefined)
  endif()
endif()

# ============================================================================================
# Unit Tests
# ============================================================================================
if(ADPT_TEST)
  include(GoogleTest)
  enable_testing()

  add_executable(adptsysc-test ${ADPT_SOURCES_TEST})
  target_compile_features(adptsysc-test PRIVATE cxx_std_20)
  # Link with gtest_main first, then other libraries
  target_link_libraries(adptsysc-test
    PRIVATE
      gtest_main
      gtest
      adptsysc_lib
    PUBLIC
     nlohmann_json::nlohmann_json
  )

  gtest_discover_tests(adptsysc-test)
endif()

# ============================================================================================
# Verbose Output
# ============================================================================================
if(ADPT_VERBOSE)
  function(PrintList title items)
    message(STATUS "--[ADPT-INFO] ${title}")
    foreach(item IN LISTS items)
      message(STATUS "    ${item}")
    endforeach()
  endfunction()

  PrintList("Main Source Files:" ${ADPT_SOURCES})
  if(ADPT_TEST)
    PrintList("Test Source Files:" ${ADPT_SOURCES_TEST})
  endif()
endif()