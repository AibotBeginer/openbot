###################################################################################################################
#
#         library components
#
###################################################################################################################

# slam_compontent
add_library(slam_component SHARED components/slam_component.cc)
target_link_libraries(slam_component ${PROJECT_NAME})
list(APPEND plugin_libs slam_component)

# install
install(TARGETS
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
