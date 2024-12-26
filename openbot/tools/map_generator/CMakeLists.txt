include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/openbot/tools/map_generator/mockamap
)

# Source files
file(GLOB_RECURSE MOCKAMAP_HDR "*.hpp" "*.h")
file(GLOB_RECURSE MOCKAMAP_SRC "*.cc" "*.cpp")

# Use qt5_add_resources for Qt5 or qt_add_resources for Qt6
qt5_add_resources(RESOURCE_SOURCES ${QRC_FILES})

set(tools_name openbot.tools.mockamap)
add_executable(${tools_name}
  ${MOCKAMAP_HDR}
  ${MOCKAMAP_SRC}
)

target_link_libraries(${tools_name} 
  openbot
  ${CYBERRT_LIBRARY})

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${tools_name} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>
)

# Install rules (if needed)
install(TARGETS ${tools_name} DESTINATION bin)