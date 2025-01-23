include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Source files
file(GLOB_RECURSE HDRS "*.hpp" "*.h")
file(GLOB_RECURSE SRCS "*.cc" "*.cpp")

# executable
set(executable_name openbot.tools.foxglove_bridge)
add_executable(${executable_name} 
  ${HDRS}
  ${SRCS}
)

target_link_libraries(${executable_name} 
  openbot
  ${FOXGLOVE_WEBSOCKET_LIBRARY}
  ${CYBERRT_LIBRARY})


# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${executable_name}  PUBLIC
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>
)

# Install rules (if needed)
install(TARGETS ${executable_name} DESTINATION bin)