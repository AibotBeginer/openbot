cmake_minimum_required(VERSION 3.15)
project(FoxgloveWebSocket CXX)

find_package(nlohmann_json REQUIRED)
find_package(websocketpp REQUIRED)


include_directories( 
	include
	${nlohmann_json_INCLUDE_DIRS}
  ${websocketpp_INCLUDE_DIRS}
)

add_library(foxglove_websocket src/base64.cpp src/parameter.cpp src/serialization.cpp src/server_factory.cpp)
target_link_libraries(foxglove_websocket nlohmann_json::nlohmann_json websocketpp::websocketpp)
set_target_properties(foxglove_websocket PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

target_include_directories(foxglove_websocket PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS foxglove_websocket DESTINATION
    ${CMAKE_INSTALL_PREFIX}/lib
)

if (MSVC)
  target_compile_options(foxglove_websocket PRIVATE /W4)
else()
  target_compile_options(foxglove_websocket PRIVATE -Wall -Wextra -Wpedantic -Wold-style-cast -Wfloat-equal)
endif()

install(
  TARGETS foxglove_websocket
  EXPORT FoxgloveExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include DESTINATION
  ${CMAKE_INSTALL_PREFIX}
)

set(FOXGLOVE_CMAKE_DIR share/foxglove/cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${PROJECT_SOURCE_DIR}/cmake/foxglove-config.cmake.in
    ${PROJECT_BINARY_DIR}/cmake/foxglove/foxglove-config.cmake
    PATH_VARS FOXGLOVE_CMAKE_DIR
    INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/foxglove
)

install(
    EXPORT FoxgloveExport
    DESTINATION share/foxglove/cmake/
    FILE FoxgloveTargets.cmake
)

install(
    FILES ${PROJECT_BINARY_DIR}/cmake/foxglove/foxglove-config.cmake
    DESTINATION share/foxglove/
)