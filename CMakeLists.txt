cmake_minimum_required(VERSION 3.27 FATAL_ERROR)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

project(
  adptsysc
  VERSION 0.1.0
  LANGUAGES CXX
  DESCRIPTION "A C++20/SystemC project for adaptive filter modeling"
  HOMEPAGE_URL "https://github.com/murumura/AdptFilter_SysC.git"
)

if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
  message(
    FATAL_ERROR
    "In-source builds are not allowed. Use a separate build directory."
  )
endif()

# ============================================================================================
# Options
# ============================================================================================
option(DOWNLOAD_EIGEN "Download Eigen" OFF)
option(ADPT_TESTONLY "Build unit tests for adptsysc only" OFF)
option(ADPT_TEST "Build unit tests for adptsysc" OFF)
option(ADPT_DBUG "Enable debugging mode" OFF)
option(ADPT_VERBOSE "Display build information" ON)
option(ADPT_TEST_DSPLIB "Enable DSP library tests" OFF)
option(ADPT_TEST_UTILS "Enable utility tests" OFF)
option(ADPT_USE_SYSTEMC_AMS "Enable SystemC AMS support" ON)
option(ADPT_USE_ASAN "Enable AddressSanitizer" OFF)
option(ADPT_USE_TSAN "Enable ThreadSanitizer" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF)

if("${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
endif()

# ============================================================================================
# Build definitions
# ============================================================================================
set(ADPT_DEFS
  SC_INCLUDE_FX
)

if(ADPT_DBUG)
  message(STATUS "--[ADPT-INFO] Debug Mode: ON (via ADPT_DBUG)")
  list(APPEND ADPT_DEFS DBUG_MODE)
endif()

message(STATUS "--[ADPT-INFO] Build type          : ${CMAKE_BUILD_TYPE}")
message(STATUS "--[ADPT-INFO] Unit tests only     : ${ADPT_TESTONLY}")
message(STATUS "--[ADPT-INFO] Unit tests          : ${ADPT_TEST}")
message(STATUS "--[ADPT-INFO] DSP library tests   : ${ADPT_TEST_DSPLIB}")
message(STATUS "--[ADPT-INFO] Utility tests       : ${ADPT_TEST_UTILS}")
message(STATUS "--[ADPT-INFO] SystemC AMS         : ${ADPT_USE_SYSTEMC_AMS}")
message(STATUS "--[ADPT-INFO] Verbose output      : ${ADPT_VERBOSE}")
message(STATUS "--[ADPT-INFO] AddressSanitizer    : ${ADPT_USE_ASAN}")
message(STATUS "--[ADPT-INFO] ThreadSanitizer     : ${ADPT_USE_TSAN}")

if(ADPT_TESTONLY AND NOT ADPT_TEST)
  message(
    FATAL_ERROR
    "\n--[ADPT-ERROR] Test-only mode requires ADPT_TEST=ON.\n"
    "Configure with:\n"
    "  -DADPT_TEST=ON\n"
    "Current ADPT_TEST=${ADPT_TEST}\n"
    "Current ADPT_TESTONLY=${ADPT_TESTONLY}\n"
  )
endif()

if(ADPT_USE_ASAN AND ADPT_USE_TSAN)
  message(
    FATAL_ERROR
    "--[ADPT-ERROR] ASan/UBSan and TSan should not be enabled together."
  )
endif()

# ============================================================================================
# Dependencies: nlohmann/json, Eigen, GoogleTest
# ============================================================================================
include(FetchContent)

FetchContent_Declare(
  json
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

message(STATUS "--[ADPT-INFO] Eigen source dir: ${eigen_SOURCE_DIR}")

if(ADPT_TEST)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
  )
  FetchContent_MakeAvailable(googletest)
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
  message(
    FATAL_ERROR
    "--[ADPT-ERROR] SystemC not found. Set SYSTEMC_HOME or install SystemC properly."
  )
endif()

message(STATUS "--[ADPT-INFO] SystemC include: ${SystemC_INCLUDE_DIRS}")
message(STATUS "--[ADPT-INFO] SystemC libs   : ${SystemC_LIBRARIES}")

if(DEFINED SystemC_VERSION)
  message(STATUS "--[ADPT-INFO] SystemC version: ${SystemC_VERSION}")

  if(SystemC_VERSION VERSION_LESS 3.0.0)
    message(
      FATAL_ERROR
      "--[ADPT-ERROR] adptsysc cycle models require SystemC >= 3.0.0; "
      "found ${SystemC_VERSION}."
    )
  endif()
endif()

# ============================================================================================
# Find SystemC AMS
# ============================================================================================
if(ADPT_USE_SYSTEMC_AMS)
  find_package(SystemCAMS REQUIRED)

  if(NOT SystemCAMS_FOUND)
    message(
      FATAL_ERROR
      "--[ADPT-ERROR] SystemC-AMS not found. "
      "Set SYSTEMC_AMS_HOME or install SystemC-AMS properly."
    )
  endif()

  message(STATUS "--[ADPT-INFO] SystemC-AMS include: ${SystemCAMS_INCLUDE_DIRS}")
  message(STATUS "--[ADPT-INFO] SystemC-AMS libs   : ${SystemCAMS_LIBRARIES}")
endif()

# ============================================================================================
# Paths
# ============================================================================================
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

# ============================================================================================
# Architecture targets
# ============================================================================================
set(ADPT_TARGETS
  SyscMemArch
  R2SdfFFTTLMArch
  SyscMemoryCycleArch
  R2SdfFFTCycleArch
  OverlapSaveFdafCycleArch
)

list(GET ADPT_TARGETS 0 ADPT_FIRST_TARGET)
message(STATUS "--[ADPT-INFO] ADPT_FIRST_TARGET: ${ADPT_FIRST_TARGET}")

foreach(TARGET IN LISTS ADPT_TARGETS)
  set(HAVE_${TARGET} 0)
endforeach()

if(NOT ADPT_TESTONLY)
  foreach(TARGET IN LISTS ADPT_TARGETS)
    set(HAVE_${TARGET} 1)
  endforeach()
endif()

# ============================================================================================
# Source files
# ============================================================================================
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
  src/adptsysc/common.hh
  src/adptsysc/arch.hh
  src/adptsysc/adptsysc.hh
  src/adptsysc/sysc-mem.hh
  src/adptsysc/object.hh
  src/adptsysc/lib.cc
  src/adptsysc/syscfx-utils.hh
  src/adptsysc/sysc-cmplxmul.hh
  src/adptsysc/sysc-shiftreg.hh
  src/adptsysc/sysc-r2sdffft.hh
  src/adptsysc/sysc-mem-cycle.hh
  src/adptsysc/sysc-cmplxmul-cycle.hh
  src/adptsysc/sysc-shiftreg-cycle.hh
  src/adptsysc/sysc-r2sdffft-cycle.hh
  src/adptsysc/ovsfdaft-cycle.hh
)

set(ADPT_SOURCES_TEST
  src/adptsysc/adptsysc-test.cc
)

if(EXISTS "${CMAKE_SOURCE_DIR}/src/adptsysc/gtest-sc-main.cc")
  list(APPEND ADPT_SOURCES_TEST
    src/adptsysc/gtest-sc-main.cc
  )
endif()

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

# ============================================================================================
# Generated per-architecture translation units
# ============================================================================================
set(ADPT_GENERATED_FILES "")

function(adpt_instantiate_templates SOURCE TARGET)
  set(options)
  set(oneValueArgs FEATURE_DEFINE)
  cmake_parse_arguments(AIT "${options}" "${oneValueArgs}" "" ${ARGN})

  set(OUTFILE "${CMAKE_BINARY_DIR}/${SOURCE}.${TARGET}.cc")
  get_filename_component(OUTDIR "${OUTFILE}" DIRECTORY)
  file(MAKE_DIRECTORY "${OUTDIR}")

  message(STATUS "--[ADPT-INFO] Instantiate ${SOURCE} for target ${TARGET}")

  set(GEN_TEXT "#define ADPT_TARGET ${TARGET}\n")

  if(AIT_FEATURE_DEFINE)
    string(APPEND GEN_TEXT "#define ${AIT_FEATURE_DEFINE} 1\n")
  endif()

  string(APPEND GEN_TEXT "#include \"${CMAKE_SOURCE_DIR}/${SOURCE}\"\n")

  file(WRITE "${OUTFILE}" "${GEN_TEXT}")

  list(APPEND ADPT_GENERATED_FILES "${OUTFILE}")
  set(ADPT_GENERATED_FILES "${ADPT_GENERATED_FILES}" PARENT_SCOPE)
endfunction()

function(adpt_register_template_source SOURCE)
  set(options)
  set(oneValueArgs FEATURE_DEFINE)
  set(multiValueArgs TARGETS)
  cmake_parse_arguments(ARTS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(NOT ARTS_TARGETS)
    message(
      FATAL_ERROR
      "adpt_register_template_source(${SOURCE}) requires TARGETS"
    )
  endif()

  foreach(TGT IN LISTS ARTS_TARGETS)
    if(ARTS_FEATURE_DEFINE)
      adpt_instantiate_templates(
        "${SOURCE}"
        "${TGT}"
        FEATURE_DEFINE "${ARTS_FEATURE_DEFINE}"
      )
    else()
      adpt_instantiate_templates("${SOURCE}" "${TGT}")
    endif()

    set(ADPT_GENERATED_FILES "${ADPT_GENERATED_FILES}" PARENT_SCOPE)
  endforeach()
endfunction()

if(NOT ADPT_TESTONLY)
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
    TARGETS
      SyscMemArch
      R2SdfFFTTLMArch
    FEATURE_DEFINE ADPT_ENABLE_SYSC_MEM
  )

  adpt_register_template_source(
    src/adptsysc/sysc-r2sdffft.cc
    TARGETS
      R2SdfFFTTLMArch
    FEATURE_DEFINE ADPT_ENABLE_R2SDF
  )

  adpt_register_template_source(
    src/adptsysc/sysc-mem-cycle.cc
    TARGETS
      SyscMemoryCycleArch
    FEATURE_DEFINE ADPT_ENABLE_SYSC_MEM_CYCLE
  )

  adpt_register_template_source(
    src/adptsysc/sysc-r2sdffft-cycle.cc
    TARGETS
      R2SdfFFTCycleArch
      OverlapSaveFdafCycleArch
    FEATURE_DEFINE ADPT_ENABLE_R2SDF_CYCLE
  )

  adpt_register_template_source(
    src/adptsysc/ovsfdaft-cycle.cc
    TARGETS
      OverlapSaveFdafCycleArch
    FEATURE_DEFINE ADPT_ENABLE_FDAF_CYCLE
  )
endif()

configure_file(
  "${CMAKE_SOURCE_DIR}/src/adptsysc/config.h.in"
  "${CMAKE_SOURCE_DIR}/src/adptsysc/config.hh"
  @ONLY
)

# ============================================================================================
# Runtime Python plotting script
# ============================================================================================
set(ADPT_PLOT_SCRIPT "${CMAKE_SOURCE_DIR}/src/adptsysc/adptplot.py")
set(ADPT_PLOT_SCRIPT_DST "${CMAKE_BINARY_DIR}/adptplot.py")

add_custom_command(
  OUTPUT "${ADPT_PLOT_SCRIPT_DST}"
  COMMAND
    "${CMAKE_COMMAND}" -E copy_if_different
    "${ADPT_PLOT_SCRIPT}"
    "${ADPT_PLOT_SCRIPT_DST}"
  DEPENDS "${ADPT_PLOT_SCRIPT}"
  COMMENT "--[ADPT] Copy adptplot.py to build directory"
)

add_custom_target(
  adpt_plot_script ALL
  DEPENDS "${ADPT_PLOT_SCRIPT_DST}"
)

# ============================================================================================
# Library: adptsysc_lib
# ============================================================================================
add_library(adptsysc_lib STATIC)

target_include_directories(
  adptsysc_lib
  PUBLIC
    "${CMAKE_SOURCE_DIR}/src"
)

target_include_directories(
  adptsysc_lib
  SYSTEM PUBLIC
    "${eigen_SOURCE_DIR}"
)

target_sources(
  adptsysc_lib
  PUBLIC
    ${ADPT_DSP_SOURCES}
)

if(NOT ADPT_TESTONLY)
  target_sources(
    adptsysc_lib
    PRIVATE
      ${ADPT_GENERATED_FILES}
      ${ADPT_SOURCES}
  )

  target_include_directories(
    adptsysc_lib
    PUBLIC
      ${SystemC_INCLUDE_DIRS}
  )

  if(ADPT_USE_SYSTEMC_AMS)
    target_include_directories(
      adptsysc_lib
      PUBLIC
        ${SystemCAMS_INCLUDE_DIRS}
    )
  endif()
endif()

target_compile_definitions(
  adptsysc_lib
  PUBLIC
    ${ADPT_DEFS}
)

target_compile_features(
  adptsysc_lib
  PUBLIC
    cxx_std_20
)

target_link_libraries(
  adptsysc_lib
  PUBLIC
    nlohmann_json::nlohmann_json
)

if(NOT ADPT_TESTONLY)
  target_link_libraries(
    adptsysc_lib
    PUBLIC
      ${SystemC_LIBRARIES}
  )

  if(ADPT_USE_SYSTEMC_AMS)
    target_link_libraries(
      adptsysc_lib
      PUBLIC
        ${SystemCAMS_LIBRARIES}
    )
  endif()
endif()

if(ADPT_USE_TSAN)
  target_compile_options(adptsysc_lib PUBLIC -fsanitize=thread)
  target_link_options(adptsysc_lib PUBLIC -fsanitize=thread)
elseif(ADPT_USE_ASAN)
  target_compile_options(
    adptsysc_lib
    PUBLIC
      -fsanitize=address
      -fsanitize=undefined
  )
  target_link_options(
    adptsysc_lib
    PUBLIC
      -fsanitize=address
      -fsanitize=undefined
  )
endif()

# ============================================================================================
# Executable: adptsysc
# ============================================================================================
if(NOT ADPT_TESTONLY)
  add_executable(
    adptsysc
    src/adptsysc/adptsysc-entry.cc
  )

  target_link_libraries(
    adptsysc
    PRIVATE
      adptsysc_lib
  )

  add_dependencies(adptsysc adpt_plot_script)
endif()

# ============================================================================================
# Unit tests
# ============================================================================================
if(ADPT_TEST)
  include(GoogleTest)
  enable_testing()

  add_executable(
    adptsysc-test
    ${ADPT_SOURCES_TEST}
  )

  target_compile_features(
    adptsysc-test
    PRIVATE
      cxx_std_20
  )

  target_link_libraries(
    adptsysc-test
    PRIVATE
      GTest::gtest_main
      GTest::gtest
      adptsysc_lib
      nlohmann_json::nlohmann_json
  )

  gtest_discover_tests(adptsysc-test)
endif()

# ============================================================================================
# Verbose output
# ============================================================================================
if(ADPT_VERBOSE)
  function(PrintList title items)
    message(STATUS "--[ADPT-INFO] ${title}")
    foreach(item IN LISTS items)
      message(STATUS "    ${item}")
    endforeach()
  endfunction()

  PrintList("Architecture targets:" "${ADPT_TARGETS}")
  PrintList("Main source files:" "${ADPT_SOURCES}")

  if(NOT ADPT_TESTONLY)
    PrintList("Generated template files:" "${ADPT_GENERATED_FILES}")
  endif()

  if(ADPT_TEST)
    PrintList("Test source files:" "${ADPT_SOURCES_TEST}")
  endif()
endif()
