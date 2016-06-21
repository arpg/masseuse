###############################################################################
# Find TINYXML2
###############################################################################
#
#  TINYXML2_FOUND
#  TINYXML2_INCLUDE_DIRS
#  TINYXML2_LIBRARIES
#
###############################################################################

find_path(TINYXML2_INCLUDE_DIRS tinyxml2.h
    HINTS $ENV{TINYXML2DIR}
    PATH_SUFFIXES include
    PATHS ~/Library/Frameworks
          /Library/Frameworks
          /usr/local
          /usr
)

find_library(TINYXML2_LIBRARIES
    tinyxml2
    HINTS $ENV{TINYXML2DIR}
    PATH_SUFFIXES lib64 lib
    PATHS ~/Library/Frameworks
          /Library/Frameworks
          /usr/local
          /usr
)

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(TINYXML2 DEFAULT_MSG TINYXML2_LIBRARIES)

mark_as_advanced(TINYXML2_LIBRAIES TINYXML2_INCLUDE_DIRS)
