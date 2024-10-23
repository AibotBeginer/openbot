# Copyright 2024 The OpenRobotic Beginner Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# TinyXML2_FOUND
# TinyXML2_INCLUDE_DIRS
# TinyXML2_LIBRARIES

# try to find the CMake config file for TinyXML2 first
if(NOT TinyXML2_FOUND)
  find_package(TinyXML2 CONFIG QUIET)
endif()
if(TinyXML2_FOUND)
  message(STATUS "Found TinyXML2 via Config file: ${TinyXML2_DIR}")
  if(NOT TINYXML2_LIBRARY)
    # in this case, we're probably using TinyXML2 version 5.0.0 or greater
    # in which case tinyxml2 is an exported target and we should use that
    if(TARGET tinyxml2)
      set(TINYXML2_LIBRARY tinyxml2)
    elseif(TARGET tinyxml2::tinyxml2)
      set(TINYXML2_LIBRARY tinyxml2::tinyxml2)
    elseif(TinyXML2_FIND_REQUIRED)
      message(FATAL_ERROR "Unable to determine target for TinyXML2")
    endif()
    list(APPEND TinyXML2_TARGETS ${TINYXML2_LIBRARY})
  else()
    # Only perform that logic once
    if(NOT TARGET tinyxml2::tinyxml2)
      # TINYXML2_LIBRARY is composed of debug;<path\to\debug.lib>;optimized;<path\to\release.lib>
      # we have to extract the appropriate component based on the current configuration.
      list(LENGTH TINYXML2_LIBRARY TINYXML_LIBRARY_LIST_LENGTH)
      if(NOT ${TINYXML_LIBRARY_LIST_LENGTH} EQUAL 4)
        message(FATAL_ERROR "Unable to extract the library file path from ${TINYXML2_LIBRARY}")
      endif()
      if(CMAKE_BUILD_TYPE MATCHES DEBUG)
        list(GET TINYXML2_LIBRARY 0 ASSERT_DEBUG)
        if(NOT ${ASSERT_DEBUG} STREQUAL "debug")
          message(FATAL_ERROR "could not parse debug library path from ${TINYXML2_LIBRARY}")
        endif()
        list(GET TINYXML2_LIBRARY 1 TINYXML2_LIBRARY_PATH)
      else()
        list(GET TINYXML2_LIBRARY 2 ASSERT_OPTIMIZED)
        if(NOT ${ASSERT_OPTIMIZED} STREQUAL "optimized")
          message(FATAL_ERROR "could not parse library path from ${TINYXML2_LIBRARY}")
        endif()
        list(GET TINYXML2_LIBRARY 3 TINYXML2_LIBRARY_PATH)
      endif()
      if(NOT EXISTS ${TINYXML2_LIBRARY_PATH})
        message(FATAL_ERROR "library file path ${TINYXML2_LIBRARY_PATH} does not exist")
      endif()
      
      add_library(tinyxml2::tinyxml2 UNKNOWN IMPORTED)
      set_property(TARGET tinyxml2::tinyxml2 PROPERTY IMPORTED_LOCATION ${TINYXML2_LIBRARY_PATH})
      set_property(TARGET tinyxml2::tinyxml2 PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${TINYXML2_INCLUDE_DIR})
      list(APPEND TinyXML2_TARGETS tinyxml2::tinyxml2)
    endif()
  endif()
else()
  find_path(TINYXML2_INCLUDE_DIR NAMES tinyxml2.h)

  find_library(TINYXML2_LIBRARY tinyxml2)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(TinyXML2 DEFAULT_MSG TINYXML2_LIBRARY TINYXML2_INCLUDE_DIR)

  mark_as_advanced(TINYXML2_INCLUDE_DIR TINYXML2_LIBRARY)

  if(NOT TARGET tinyxml2::tinyxml2)
    add_library(tinyxml2::tinyxml2 UNKNOWN IMPORTED)
    set_property(TARGET tinyxml2::tinyxml2 PROPERTY IMPORTED_LOCATION ${TINYXML2_LIBRARY})
    set_property(TARGET tinyxml2::tinyxml2 PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${TINYXML2_INCLUDE_DIR})
    list(APPEND TinyXML2_TARGETS tinyxml2::tinyxml2)
  endif()
endif()

# Set mixed case INCLUDE_DIRS and LIBRARY variables from upper case ones.
if(NOT TinyXML2_INCLUDE_DIRS)
  set(TinyXML2_INCLUDE_DIRS ${TINYXML2_INCLUDE_DIR})
endif()
if(NOT TinyXML2_LIBRARIES)
  set(TinyXML2_LIBRARIES ${TINYXML2_LIBRARY})
endif()
