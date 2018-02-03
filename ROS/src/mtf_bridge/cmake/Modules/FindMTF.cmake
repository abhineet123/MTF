###############################################################################
# Find MTF
#
# This sets the following variables:
# MTF_FOUND - True if MTF was found.
# MTF_INCLUDE_DIRS - Directories containing the MTF include files.
# MTF_LIBRARIES - Libraries needed to use MTF.
# MTF_DEFINITIONS - Compiler flags for MTF.
# If MTF_USE_STATIC is specified and then look for static libraries ONLY else
# look for shared ones

set(MTF_BASE_NAME mtf)

if(MTF_USE_STATIC)
  set(MTF_RELEASE_NAME ${MTF_BASE_NAME}_s)
  set(MTF_DEBUG_NAME ${MTF_BASE_NAME}_s-gd)
else(MTF_USE_STATIC)
  set(MTF_RELEASE_NAME ${MTF_BASE_NAME})
  set(MTF_DEBUG_NAME ${MTF_BASE_NAME}-gd)
endif(MTF_USE_STATIC)

find_package(PkgConfig QUIET)
if (MTF_FIND_VERSION)
    pkg_check_modules(PC_MTF mtf>=${MTF_FIND_VERSION})
else(MTF_FIND_VERSION)
    pkg_check_modules(PC_MTF mtf)
endif(MTF_FIND_VERSION)

set(MTF_DEFINITIONS ${PC_MTF_CFLAGS_OTHER})
message(STATUS "MTF_ROOT: " "$ENV{MTF_ROOT}")
find_path(MTF_INCLUDE_DIR mtf/mtf.h
          HINTS ${PC_MTF_INCLUDEDIR} ${PC_MTF_INCLUDE_DIRS} "${MTF_ROOT}" "$ENV{MTF_ROOT}"
          PATHS "$ENV{PROGRAMFILES}/MTF" "$ENV{PROGRAMW6432}/MTF" 
          PATH_SUFFIXES include)

find_library(MTF_LIBRARY
             NAMES ${MTF_RELEASE_NAME}
             HINTS ${PC_MTF_LIBDIR} ${PC_MTF_LIBRARY_DIRS} "${MTF_ROOT}" "$ENV{MTF_ROOT}"
             PATHS "$ENV{PROGRAMFILES}/MTF" "$ENV{PROGRAMW6432}/MTF" 
	     PATH_SUFFIXES lib)

find_library(MTF_LIBRARY_DEBUG 
             NAMES ${MTF_DEBUG_NAME} ${MTF_RELEASE_NAME}
	     HINTS ${PC_MTF_LIBDIR} ${PC_MTF_LIBRARY_DIRS} "${MTF_ROOT}" "$ENV{MTF_ROOT}"
	     PATHS "$ENV{PROGRAMFILES}/MTF" "$ENV{PROGRAMW6432}/MTF" 
	     PATH_SUFFIXES lib)

if(NOT MTF_LIBRARY_DEBUG)
  set(MTF_LIBRARY_DEBUG ${MTF_LIBRARY})
endif(NOT MTF_LIBRARY_DEBUG)

set(MTF_INCLUDE_DIRS ${MTF_INCLUDE_DIR})
set(MTF_LIBRARIES optimized ${MTF_LIBRARY} debug ${MTF_LIBRARY_DEBUG})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MTF DEFAULT_MSG MTF_LIBRARY MTF_INCLUDE_DIR)

mark_as_advanced(MTF_LIBRARY MTF_LIBRARY_DEBUG MTF_INCLUDE_DIR)

if(MTF_FOUND)
  message(STATUS "MTF found (include: ${MTF_INCLUDE_DIRS}, lib: ${MTF_LIBRARIES})")
  if(MTF_USE_STATIC)
    add_definitions(-DMTF_STATIC)
  endif(MTF_USE_STATIC)
endif(MTF_FOUND)
