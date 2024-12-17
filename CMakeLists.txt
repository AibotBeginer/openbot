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

cmake_minimum_required(VERSION 3.5)

project(openbot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OPENBOT_MAJOR_VERSION 1)
set(OPENBOT_MINOR_VERSION 0)
set(OPENBOT_PATCH_VERSION 1)
set(OPENBOT_VERSION ${OPENBOT_MAJOR_VERSION}.${OPENBOT_MINOR_VERSION}.${OPENBOT_PATCH_VERSION})
set(OPENBOT_SOVERSION ${OPENBOT_MAJOR_VERSION}.${OPENBOT_MINOR_VERSION})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

option(BUILD_GRPC "Build openbot gRPC support" true)
option(BUILD_DOCS "Build openbot docs support" OFF)
option(BUILD_TOOLS "Build openbot tools support" OFF)
option(BUILD_PROMETHEUS "Build prometheus monitoring support" false)
option(BUILD_TEST "Build openbot unit test" OFF)

include("${PROJECT_SOURCE_DIR}/cmake/functions.cmake")
google_initialize_openbot_project()
google_enable_testing()

# Set a default install prefix if not specified by the user
if(NOT CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "/opt/openbot" CACHE PATH "Default install prefix" FORCE)
endif()

# thirdparty
add_subdirectory(thirdparty)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/tf2/include)

find_package(absl REQUIRED)
set(BOOST_COMPONENTS iostreams graph program_options system)

find_package(CYBERRT REQUIRED)
find_package(FastRTPS REQUIRED)
find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})
find_package(Ceres REQUIRED)
# find_package(G2O REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Lua REQUIRED)
find_package(Protobuf REQUIRED)
find_package(TinyXML2 REQUIRED)
# find_package(behaviortree_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OMPL REQUIRED)
find_package(OSQP REQUIRED)
find_package(octomap REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(CYBERRT REQUIRED)

if (${BUILD_GRPC})
  find_package(gRPC REQUIRED)
  message(STATUS "Using gRPC ${gRPC_VERSION}")
endif()

if(${BUILD_PROMETHEUS})
  find_package( ZLIB REQUIRED )
endif()

# Only build the documentation if we can find Sphinx.
if(${BUILD_DOCS})
  find_package(Sphinx)
  if(SPHINX_FOUND)
    add_subdirectory("docs")
  endif()
endif()

# Install catkin package.xml
install(FILES package.xml DESTINATION share/openbot)

set(OPENBOT_CONFIGURATION_FILES_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/openbot/config
  CACHE PATH ".lua configuration files directory")

install(DIRECTORY config DESTINATION share/openbot/)
install(DIRECTORY cmake DESTINATION share/openbot/)

file(GLOB_RECURSE ALL_LIBRARY_HDRS "openbot/*.hpp" "openbot/*.h")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "openbot/*.cc" "openbot/*.cpp")
file(GLOB_RECURSE TEST_LIBRARY_HDRS "openbot/fake_*.hpp" "openbot/*_test_helpers*.hpp")
file(GLOB_RECURSE TEST_LIBRARY_SRCS "openbot/fake_*.cc" "openbot/fake_*.cpp" "openbot/*_test_helpers*.cc" "openbot/*_test_helpers*.cpp")
file(GLOB_RECURSE ALL_TOOLS_HDRS "openbot/tools/*.hpp" "openbot/tools/*.h")
file(GLOB_RECURSE ALL_TOOLS_SRCS "openbot/tools/*.cc" "openbot/tools*.cpp")
file(GLOB_RECURSE ALL_TESTS "openbot/*_test.cc" "openbot/*_test.cpp")
file(GLOB_RECURSE ALL_EXECUTABLES "openbot/*_main.cc")

# Remove dotfiles/-folders that could potentially pollute the build.
file(GLOB_RECURSE ALL_DOTFILES ".*/*")
if (ALL_DOTFILES)
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_DOTFILES})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_DOTFILES})
  list(REMOVE_ITEM TEST_LIBRARY_HDRS ${ALL_DOTFILES})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_DOTFILES})
  list(REMOVE_ITEM ALL_TESTS ${ALL_DOTFILES})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_DOTFILES})
endif()

list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_EXECUTABLES})
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_TESTS})
list(REMOVE_ITEM ALL_LIBRARY_HDRS ${TEST_LIBRARY_HDRS})
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${TEST_LIBRARY_SRCS})
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_TOOLS_HDRS})
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_TOOLS_SRCS})
file(GLOB_RECURSE ALL_GRPC_FILES "openbot/bridge/common/grpc/*")

if (NOT ${BUILD_GRPC})
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM TEST_LIBRARY_HDRS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_TESTS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_GRPC_FILES})
endif()

file(GLOB_RECURSE ALL_PROTOS "openbot/*.proto")
file(GLOB_RECURSE ALL_GRPC_SERVICES "openbot/*_service.proto")
list(REMOVE_ITEM ALL_PROTOS ${ALL_GRPC_SERVICES})

set(INSTALL_SOURCE_HDRS ${ALL_LIBRARY_HDRS} ${TEST_LIBRARY_HDRS})
file(GLOB_RECURSE INTERNAL_HDRS "openbot/*/internal/*.hpp")
list(REMOVE_ITEM INSTALL_SOURCE_HDRS ${INTERNAL_HDRS})

# TODO(cschuet): Move proto compilation to separate function.
set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)

find_program(PROTOBUF_PROTOC_EXECUTABLE protoc)
if(NOT PROTOBUF_PROTOC_EXECUTABLE)
  message(STATUS, "protoc not found. Please install protobuf and make sure it's in your PATH.")
endif()

if(${BUILD_GRPC})
  find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin)
  if(NOT GRPC_CPP_PLUGIN)
    message(FATAL_ERROR "grpc_cpp_plugin not found. Please install gRPC and make sure it's in your PATH.")
  endif()
  set(ALL_GRPC_SERVICE_SRCS)
  set(ALL_GRPC_SERVICE_HDRS)
  foreach(ABS_FIL ${ALL_GRPC_SERVICES})
    file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
    get_filename_component(DIR ${REL_FIL} DIRECTORY)
    get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

    list(APPEND ALL_GRPC_SERVICE_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.grpc.pb.cc")
    list(APPEND ALL_GRPC_SERVICE_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.grpc.pb.h")
    list(APPEND ALL_GRPC_SERVICE_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
    list(APPEND ALL_GRPC_SERVICE_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

    add_custom_command(
      OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
             "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
             "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.grpc.pb.cc"
             "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.grpc.pb.h"
      COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS --grpc_out=${PROJECT_BINARY_DIR}
        --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
        --cpp_out  ${PROJECT_BINARY_DIR}
        -I ${PROJECT_SOURCE_DIR}
        ${ABS_FIL}
      DEPENDS ${ABS_FIL}
      COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
      VERBATIM
    )
  endforeach()
  set_source_files_properties(${ALL_GRPC_SERVICE_SRCS} ${ALL_GRPC_SERVICE_HDRS} PROPERTIES GENERATED TRUE)
  list(APPEND ALL_LIBRARY_HDRS ${ALL_GRPC_SERVICE_HDRS})
  list(APPEND ALL_LIBRARY_SRCS ${ALL_GRPC_SERVICE_SRCS})
endif()

foreach(ABS_FIL ${ALL_PROTOS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  list(APPEND ALL_PROTO_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

  add_custom_command(
    OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
           "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${PROJECT_BINARY_DIR} -I
      ${PROJECT_SOURCE_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
  )
endforeach()

# foreach(HDR ${ALL_PROTO_HDRS})
#   file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
#   get_filename_component(DIR ${REL_FIL} DIRECTORY)
#   get_filename_component(ABSOLUTE_PATH "${DIR}" ABSOLUTE)
#   string(REPLACE "/" ";" PATH_LIST "${ABSOLUTE_PATH}")
#   list(LENGTH PATH_LIST PATH_LENGTH)
#   if(PATH_LENGTH GREATER_EQUAL 4)
#       math(EXPR START_INDEX "${PATH_LENGTH} - 4")
#       list(SUBLIST PATH_LIST ${START_INDEX} 4 LAST_FOUR)
#       string(REPLACE ";" "/" LAST_FOUR_PATH "${LAST_FOUR}")
#   else()
#       message(STATUS "Path is too short to extract last four directories.")
#   endif()
    
#   install(
#     FILES ${HDR}
#     DESTINATION include/${LAST_FOUR_PATH}
#   )
# endforeach()

set_source_files_properties(${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS} PROPERTIES GENERATED TRUE)
list(APPEND ALL_LIBRARY_HDRS ${ALL_PROTO_HDRS})
list(APPEND ALL_LIBRARY_SRCS ${ALL_PROTO_SRCS})

set(INSTALL_GENERATED_HDRS ${ALL_PROTO_HDRS} ${ALL_GRPC_SERVICE_HDRS})

add_library(${PROJECT_NAME} SHARED ${ALL_LIBRARY_HDRS} ${ALL_LIBRARY_SRCS})
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PROTOBUF_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PROTOBUF_LIBRARY} 
  absl::algorithm
  absl::base
  absl::debugging
  absl::flat_hash_map
  absl::memory
  absl::meta
  absl::numeric
  absl::str_format
  absl::strings
  absl::synchronization
  absl::time
  absl::utility 
)

if(${BUILD_GRPC})
  target_link_libraries(${PROJECT_NAME} PUBLIC gRPC::grpc++ gRPC::grpc)
endif()

configure_file(
  ${PROJECT_SOURCE_DIR}/openbot/common/config.hpp.cmake
  ${PROJECT_BINARY_DIR}/openbot/common/config.hpp)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/openbot-config-version.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/openbot-config-version.cmake @ONLY)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/openbot-config-version.cmake"
      DESTINATION "share/openbot")

# Generate source file with version definitions.
include("${PROJECT_SOURCE_DIR}/cmake/GenerateVersionDefinitions.cmake")

if (BUILD_TOOLS)
  add_subdirectory(openbot/tools)
endif()

# target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${behaviortree_cpp_INCLUDE_DIR}")
# target_link_libraries(${PROJECT_NAME} PUBLIC behaviortree_cpp)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${CYBERRT_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${CYBERRT_LIBRARY})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${FastRTPS_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${FastRTPS_LIBRARY})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${EIGEN3_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${EIGEN3_LIBRARIES})

# target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${CERES_INCLUDE_DIR}")
# target_link_libraries(${PROJECT_NAME} PUBLIC ${CERES_LIBRARIES})

# target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY}")
# target_link_libraries(${PROJECT_NAME} PUBLIC ${G2O_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${LUA_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${LUA_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${TinyXML2_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${TinyXML2_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${CERES_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${CERES_LIBRARY})

set(GLOG_LIBRARY glog)
target_link_libraries(${PROJECT_NAME} PUBLIC ${GLOG_LIBRARY})
target_link_libraries(${PROJECT_NAME} PUBLIC gflags)

# target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${class_loader_INCLUDE_DIR}")
# target_link_libraries(${PROJECT_NAME} PUBLIC class_loader)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${PCL_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${OpenCV_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${OMPL_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${OMPL_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${OSQP_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${OSQP_LIBRARY})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${OCTOMAP_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${OCTOMAP_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${YAML_CPP_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp)

# target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${library_loader_INCLUDE_DIR}")
# target_link_libraries(${PROJECT_NAME} PUBLIC library_loader)
target_link_libraries(${PROJECT_NAME} PUBLIC tf2)

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")

# compontent
add_subdirectory(${PROJECT_SOURCE_DIR}/openbot/bridge)
add_subdirectory(${PROJECT_SOURCE_DIR}/openbot/control)
add_subdirectory(${PROJECT_SOURCE_DIR}/openbot/drivers)
add_subdirectory(${PROJECT_SOURCE_DIR}/openbot/planning)


# set(TEST_LIB openbot_test_library)
# add_library(${TEST_LIB} ${TEST_LIBRARY_HDRS} ${TEST_LIBRARY_SRCS})
# target_include_directories(${TEST_LIB} SYSTEM PRIVATE "${GMOCK_INCLUDE_DIRS}")
# # Needed for dynamically linked GTest on Windows.
# target_link_libraries(${TEST_LIB} PUBLIC ${GMOCK_LIBRARY})
# target_link_libraries(${TEST_LIB} PUBLIC ${PROJECT_NAME})
# set_target_properties(${TEST_LIB} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

if (BUILD_TEST)
  foreach(ABS_FIL ${ALL_TESTS})
    file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
    get_filename_component(DIR ${REL_FIL} DIRECTORY)
    get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
    # Replace slashes as required for CMP0037.
    string(REPLACE "/" "." TEST_TARGET_NAME "${DIR}/${FIL_WE}")
    google_test("${TEST_TARGET_NAME}" ${ABS_FIL})
    if(${BUILD_GRPC})
      target_link_libraries("${TEST_TARGET_NAME}" PUBLIC gRPC::grpc++ gRPC::grpc ${Protobuf_LIBRARIES})
    endif()
    target_link_libraries("${TEST_TARGET_NAME}" PUBLIC ${TEST_LIB})
  endforeach()
endif()

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT OpenbotExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

foreach(HDR ${INSTALL_SOURCE_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES ${HDR}
    DESTINATION include/${DIR}
  )
endforeach()

foreach(HDR ${INSTALL_GENERATED_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_BINARY_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES ${HDR}
    DESTINATION include/${DIR}
  )
endforeach()

set(OPENBOT_CMAKE_DIR share/openbot/cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/openbot-config.cmake.in
  ${PROJECT_BINARY_DIR}/cmake/openbot/openbot-config.cmake
  PATH_VARS OPENBOT_CMAKE_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/openbot
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  EXPORT OpenbotExport
  DESTINATION share/openbot/cmake/
  FILE OpenbotTargets.cmake
)

install(
  FILES ${PROJECT_BINARY_DIR}/cmake/openbot/openbot-config.cmake
  DESTINATION share/openbot/
)

# conf dag launch
file(GLOB_RECURSE ALL_OPENBOT_DAGS "openbot/*.dag")
file(GLOB_RECURSE ALL_OPENBOT_CONFS "openbot/*.pb.txt" "openbot/*.conf")
file(GLOB_RECURSE ALL_OPENBOT_LAUNCHS "openbot/*.launch*")

# install dag files
foreach(DAG ${ALL_OPENBOT_DAGS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${DAG})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES ${DAG}
      DESTINATION share/${DIR}
  )
endforeach()

# install conf files
foreach(CONF ${ALL_OPENBOT_CONFS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${CONF})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES ${CONF}
      DESTINATION share/${DIR}
  )
endforeach()

# install launch files
foreach(LAUNCH ${ALL_OPENBOT_LAUNCHS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${LAUNCH})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES ${LAUNCH}
      DESTINATION share/${DIR}
  )
endforeach()
