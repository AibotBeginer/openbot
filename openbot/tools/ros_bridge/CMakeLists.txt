include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/common
  ${CMAKE_CURRENT_SOURCE_DIR}/converter
  ${CMAKE_CURRENT_SOURCE_DIR}/converter_base
  ${CMAKE_CURRENT_SOURCE_DIR}/openbot/tools
)

# Source files
file(GLOB_RECURSE HDRS "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp" "*.h")
file(GLOB_RECURSE SRCS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" "*.cc")


# Add executable target
set(executable_name openbot.tools.ros_bridge)
add_executable(${executable_name} ${HDRS} ${SRCS})

# Link libraries
target_link_libraries(${executable_name} openbot)
target_link_libraries(${executable_name} ${CYBERRT_LIBRARY})

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${executable_name}  PUBLIC
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>
)

# Install rules (if needed)
install(TARGETS ${executable_name} DESTINATION bin)
