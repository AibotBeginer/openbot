# set(CMAKE_INCLUDE_CURRENT_DIR ON)
# set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTOUIC ON)
# set(CMAKE_AUTORCC ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/visualizer
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Source files
file(GLOB_RECURSE VISUALIZER_HDR "*.hpp" "*.h")
file(GLOB_RECURSE VISUALIZER_SRC "*.cc" "*.cpp")
file(GLOB_RECURSE RESOURCE_FILES "images/*" "shaders/*")
file(GLOB_RECURSE UI_FILES "*.ui")
file(GLOB_RECURSE QRC_FILES "*.qrc")

# Qt5 wrap UI
qt5_wrap_ui(UI_HEADERS ${UI_FILES})
qt5_add_resources(QRC_SOURCES ${QRC_FILES})

# Add library
set(library_name visualizer_lib)
add_library(visualizer_lib SHARED
  ${VISUALIZER_HDR}
  ${VISUALIZER_SRC}
  ${UI_HEADERS}
  ${QRC_SOURCES}
)

target_link_libraries(${library_name}
    Qt5::Core
    Qt5::Gui
    Qt5::OpenGL
    Qt5::Widgets
    ${CYBERRT_LIBRARY}
    openbot)

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${library_name} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/openbot/tools/visualizer>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)


# Add executable
add_executable(cyber_visualizer visualizer/main.cc)  
target_link_libraries(cyber_visualizer
    ${library_name}
    ${CYBERRT_LIBRARY}
    ${UI_HEADERS}
    openbot
    Qt5::Core
    Qt5::Gui
    Qt5::OpenGL
    Qt5::Widgets
    pthread)

# Install rules (if needed)
install(TARGETS cyber_visualizer DESTINATION bin)