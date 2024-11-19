set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)


find_package(Qt5 REQUIRED COMPONENTS OpenGL Core Gui Widgets)
find_package(OpenGL REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/openbot/tools/visualizer/visualizer
)

# Source files
file(GLOB_RECURSE VISUALIZER_HDR "visualizer/*.hpp" "visualizer/*.h")
file(GLOB_RECURSE VISUALIZER_SRC "visualizer/*.cc" "visualizer/*.cpp")
file(GLOB_RECURSE RESOURCE_FILES "visualizer/images/*" "visualizer/shaders/*")
file(GLOB_RECURSE UI_FILES "visualizer/uis/*.ui")
file(GLOB_RECURSE QRC_FILES "visualizer/*.qrc")

add_executable(openbot_visualizer
  ${VISUALIZER_HDR}
  ${VISUALIZER_SRC}
  ${UI_HEADERS}
  ${QRC_SOURCES}
)

target_link_libraries(openbot_visualizer 
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
  Qt5::OpenGL
  Qt5::Widgets
  OpenGL::GL
  openbot
  ${CYBERRT_LIBRARY})

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(openbot_visualizer PUBLIC
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/openbot/tools/visualizer>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/openbot/tools/visualizer>
  $<INSTALL_INTERFACE:include>
)

# Install rules (if needed)
install(TARGETS openbot_visualizer DESTINATION bin)

install(DIRECTORY visualizer/images DESTINATION share/openbot/)
install(DIRECTORY visualizer/shaders DESTINATION share/openbot/)