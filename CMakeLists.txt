cmake_minimum_required(VERSION 3.27 FATAL_ERROR)
cmake_policy(SET CMP0167 NEW)

project(
  adptsysc
  VERSION 0.1.0
  LANGUAGES CXX
  DESCRIPTION "A C++20/SystemC OFDM implementation"
  HOMEPAGE_URL "https://github.com/murumura/AdptFilter_SysC.git"
)

if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
  message(FATAL_ERROR "In-source builds are not allowed. Use a separate build directory.")
endif()

# ============================================================================================
# Options
# ============================================================================================
option(DOWNLOAD_EIGEN "Download EIGEN (requires CMake 3.11+)" OFF)
option(ADPT_TEST "Build unit tests for adptsysc" OFF)
option(ADPT_DBUG "Enable Debugging Mode" OFF)
option(ADPT_VERBOSE "Display build information" ON)
option(ADPT_TEST_SIGNAL "Enable test signal processing functions" OFF)
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
  list(APPEND ADPT_DEFS -Dadptsysc_DBUG_MODE)
endif()

message(STATUS "--[ADPT-INFO] Unit tests : ${ADPT_TEST}")
message(STATUS "--[ADPT-INFO] Verbose output: ${ADPT_VERBOSE}")

if("${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
endif()

# ============================================================================================
# Dependencies: Eigen, GoogleTest
# ============================================================================================
include(FetchContent)

FetchContent_Declare(
  Eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
  GIT_SHALLOW TRUE
)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(Eigen)
message(STATUS "--[ADPT-INFO] Success. Populated Eigen3 at ${eigen_SOURCE_DIR}")
if(ADPT_TEST)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.13.0
  )
  FetchContent_MakeAvailable(googletest)
endif()

# ============================================================================================
# Find SystemC
# ============================================================================================

if(NOT DEFINED SystemC_DIR AND DEFINED ENV{SYSTEMC_HOME})
  set(SystemC_DIR "$ENV{SYSTEMC_HOME}/lib-linux64/cmake/SystemC")
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
  if(NOT DEFINED SystemCAMS_DIR AND DEFINED ENV{SYSTEMC_AMS_HOME})
    set(SystemCAMS_DIR "$ENV{SYSTEMC_AMS_HOME}/lib-linux64/cmake/SystemC-AMS")
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
# (e.g. LMS) as its type parameter. Since we suport more than 10
# targets, compiling a single source file for all the targets is very
# slow.
#
# As a workaround, we create a .cc file for each target and spawn many
# compiler instances. This is hacky but greatly reduces compile time
# on a multicore machine.
list(APPEND ADPT_TARGETS
  LMS
)
list(APPEND ADPT_TEMPLATE_FILES
  src/adptsysc/lms.cc
  src/adptsysc/cmdline.cc
)

set(ADPT_SOURCES
  src/adptsysc/adptsysc.hh
  src/adptsysc/adptsysc.cc
  src/adptsysc/integers.hh
  src/adptsysc/common.hh
  src/adptsysc/common.cc
)

set(ADPT_SOURCES_TEST src/adptsysc/adptsysc-test.cc)
if(ADPT_TEST_SIGNAL)
  list(APPEND ADPT_SOURCES_TEST src/adptsysc/test-signal.cc)
endif()
if(ADPT_TEST_UTILS)
  list(APPEND ADPT_SOURCES_TEST src/adptsysc/test-utils.cc)
endif()

function(adpt_instantiate_templates SOURCE TARGET)
  set(OUTFILE ${CMAKE_BINARY_DIR}/${SOURCE}.${TARGET}.cc)
  if(NOT EXISTS ${OUTFILE})
    file(WRITE ${OUTFILE}
      "#define ADPT_TARGET ${TARGET}\n"
      "#include \"${CMAKE_SOURCE_DIR}/${SOURCE}\"\n"
    )
  endif()
  list(APPEND ADPT_GENERATED_FILES ${OUTFILE})
  set(ADPT_GENERATED_FILES ${ADPT_GENERATED_FILES} PARENT_SCOPE)
endfunction()

set(ADPT_GENERATED_FILES "")
foreach(SOURCE IN LISTS ADPT_TEMPLATE_FILES)
  foreach(TARGET IN LISTS ADPT_TARGETS)
    adpt_instantiate_templates(${SOURCE} ${TARGET})
  endforeach()
endforeach()

# ============================================================================================
# Target: adptsysc_lib
# ============================================================================================
# Define the library once
add_library(adptsysc_lib STATIC)

# Add generated template files
target_sources(adptsysc_lib PRIVATE ${ADPT_GENERATED_FILES})

# Add hand-written source files
target_sources(adptsysc_lib PRIVATE ${ADPT_SOURCES})

message(STATUS "    ${ADPT_GENERATED_FILES}")

target_include_directories(adptsysc_lib
  PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${SystemC_INCLUDE_DIRS}
    ${SystemCAMS_INCLUDE_DIRS}   
)

# treat Eigen as a system library, suppressing its warnings
target_include_directories(adptsysc_lib
  SYSTEM PUBLIC
  ${Eigen_SOURCE_DIR}
)

target_compile_definitions(adptsysc_lib PUBLIC ${ADPT_DEFS})
target_compile_features(adptsysc_lib PUBLIC cxx_std_20)
target_link_libraries(adptsysc_lib
  PUBLIC
    ${SystemC_LIBRARIES}
    ${SystemCAMS_LIBRARIES}
)

# ============================================================================================
# Executable: adptsysc
# ============================================================================================
add_executable(adptsysc src/adptsysc/adptsysc.cc)
target_link_libraries(adptsysc PRIVATE adptsysc_lib)
if(ADPT_USE_TSAN)
  target_link_options(adptsysc PRIVATE -fsanitize=thread)
  target_compile_options(adptsysc PRIVATE -fsanitize=thread)
endif()

if(ADPT_USE_ASAN)
  target_link_options(adptsysc PRIVATE -fsanitize=address -fsanitize=undefined)
  target_compile_options(adptsysc PRIVATE -fsanitize=address -fsanitize=undefined)
endif()

# ============================================================================================
# Unit Tests
# ============================================================================================
if(ADPT_TEST)
  include(GoogleTest)
  enable_testing()

  add_executable(adptsysc-test ${ADPT_SOURCES_TEST})
  target_link_libraries(adptsysc-test
    PRIVATE adptsysc_lib
    PRIVATE gtest_main
    PRIVATE ${SystemC_LIBRARIES}
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

  PrintList("Main Source Files:" ADPT_SOURCES)
  if(ADPT_TEST)
    PrintList("Test Source Files:" ADPT_SOURCES_TEST)
  endif()
endif()