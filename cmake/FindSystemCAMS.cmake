# cmake/FindSystemCAMS.cmake
# Tries to locate SystemC AMS library and headers

find_path(SystemCAMS_INCLUDE_DIR
  NAMES systemc-ams.h
  HINTS ENV SYSTEMC_AMS_HOME
  PATH_SUFFIXES include include/systemc-ams
)

find_library(SystemCAMS_LIBRARY
  NAMES systemc-ams
  HINTS ENV SYSTEMC_AMS_HOME
  PATH_SUFFIXES lib lib-linux lib-linux64
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SystemCAMS
  REQUIRED_VARS SystemCAMS_LIBRARY SystemCAMS_INCLUDE_DIR
  VERSION_VAR SystemCAMS_VERSION
)

if(SystemCAMS_FOUND)
  set(SystemCAMS_LIBRARIES ${SystemCAMS_LIBRARY})
  set(SystemCAMS_INCLUDE_DIRS ${SystemCAMS_INCLUDE_DIR})
endif()
