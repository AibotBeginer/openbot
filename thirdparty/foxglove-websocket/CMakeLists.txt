cmake_minimum_required(VERSION 3.5)
project(foxglove_websocket)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

find_package(nlohmann_json REQUIRED)
find_package(websocketpp REQUIRED)

include_directories( 
	include
	${nlohmann_json_INCLUDE_DIRS}
  ${websocketpp_INCLUDE_DIRS}
)

# export user definitions
add_library(foxglove_websocket SHARED
  include/foxglove/websocket/base64.hpp
  include/foxglove/websocket/callback_queue.hpp
  include/foxglove/websocket/common.hpp
  include/foxglove/websocket/parameter.hpp
  include/foxglove/websocket/regex_utils.hpp
  include/foxglove/websocket/serialization.hpp
  include/foxglove/websocket/server_factory.hpp
  include/foxglove/websocket/websocket_client.hpp
  include/foxglove/websocket/websocket_logging.hpp
  include/foxglove/websocket/websocket_notls.hpp
  include/foxglove/websocket/websocket_server.hpp
  include/foxglove/websocket/websocket_tls.hpp
	src/base64.cpp
	src/parameter.cpp
	src/serialization.cpp
	src/server_factory.cpp
)
target_link_libraries(foxglove_websocket 
  nlohmann_json::nlohmann_json 
  websocketpp::websocketpp)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS ${PROJECT_NAME} DESTINATION
    ${CMAKE_INSTALL_PREFIX}/lib
)

install(DIRECTORY include DESTINATION
  ${CMAKE_INSTALL_PREFIX}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT FoxgloveWebsocketExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

set(FOXGLOVE_WEBSOCKET_CMAKE_DIR share/foxglove_websocket/cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${PROJECT_SOURCE_DIR}/cmake/foxglove_websocket-config.cmake.in
  ${PROJECT_BINARY_DIR}/cmake/foxglove_websocket/foxglove_websocket-config.cmake
  PATH_VARS FOXGLOVE_WEBSOCKET_CMAKE_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/foxglove_websocket
)

install(
  EXPORT FoxgloveWebsocketExport
  DESTINATION share/foxglove_websocket/cmake/
  FILE FoxgloveWebsocketTargets.cmake
)

install(
  FILES ${PROJECT_BINARY_DIR}/cmake/foxglove_websocket/foxglove_websocket-config.cmake
  DESTINATION share/foxglove_websocket/
)

