# Set hints for where to find SystemC
set(SystemC_ROOT "/usr/local/systemc-3.0.1")

find_path(SYSTEMC_INCLUDE_DIR systemc.h
    HINTS ${SystemC_ROOT}/include
    PATHS /usr/include /usr/local/include
    DOC "SystemC header files directory"
)

find_library(SYSTEMC_LIBRARY 
    NAMES systemc
    HINTS ${SystemC_ROOT}/lib
    PATHS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib
    DOC "SystemC library"
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SystemC DEFAULT_MSG 
    SYSTEMC_LIBRARY 
    SYSTEMC_INCLUDE_DIR
)

if(SystemC_FOUND)
    set(SystemC_INCLUDE_DIRS ${SYSTEMC_INCLUDE_DIR})
    set(SystemC_LIBRARIES ${SYSTEMC_LIBRARY})
    message(STATUS "Found SystemC include: ${SystemC_INCLUDE_DIRS}")
    message(STATUS "Found SystemC libs: ${SystemC_LIBRARIES}")
endif()