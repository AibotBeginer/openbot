# FindCeres.cmake

# Locate Ceres library
# This module defines the following variables:
#  CERES_FOUND - True if the library was found
#  CERES_INCLUDE_DIRS - Include directories for Ceres
#  CERES_LIBRARIES - Libraries to link against

find_path(CERES_INCLUDE_DIR
  NAMES ceres/ceres.h
  PATHS
    /usr/local/include
    /usr/include
    ${CMAKE_PREFIX_PATH}/include
)

find_library(CERES_LIBRARY
  NAMES ceres
  PATHS
    /usr/local/lib
    /usr/lib
    ${CMAKE_PREFIX_PATH}/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Ceres DEFAULT_MSG
  CERES_INCLUDE_DIR CERES_LIBRARY)

if (CERES_FOUND)
  set(CERES_INCLUDE_DIRS ${CERES_INCLUDE_DIR})
  set(CERES_LIBRARIES ${CERES_LIBRARY})
endif()
