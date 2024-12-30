# library()
set(library_name nav_behavior_tree)
add_library(${library_name} SHARED behavior_tree/behavior_tree_engine.cc)
target_link_libraries(${library_name} ${PROJECT_NAME})

# library(rate_controller)
add_library(nav_rate_controller_bt_node SHARED behavior_tree/plugins/decorator/rate_controller.cc)
target_link_libraries(nav_rate_controller_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_rate_controller_bt_node)

# library(speed_controller)
add_library(nav_speed_controller_bt_node SHARED behavior_tree/plugins/decorator/speed_controller.cc)
target_link_libraries(nav_speed_controller_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_speed_controller_bt_node)

# install
install(TARGETS 
  ${library_name}
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
