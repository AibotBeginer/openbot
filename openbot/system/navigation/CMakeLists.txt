# library()
set(library_name nav_behavior_tree)
add_library(${library_name} SHARED behavior_tree/behavior_tree_engine.cc)
target_link_libraries(${library_name} ${PROJECT_NAME})

# library(pipeline_sequence)
add_library(nav_pipeline_sequence_bt_node SHARED behavior_tree/plugins/control/pipeline_sequence.cc)
target_link_libraries(nav_pipeline_sequence_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_pipeline_sequence_bt_node)

# library(recovery_node)
add_library(nav_recovery_node_bt_node SHARED behavior_tree/plugins/control/recovery_node.cc)
target_link_libraries(nav_recovery_node_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_recovery_node_bt_node)

# library(round_robin_node)
add_library(nav_round_robin_node_bt_node SHARED behavior_tree/plugins/control/round_robin_node.cc)
target_link_libraries(nav_round_robin_node_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_round_robin_node_bt_node)

# library(distance_controller)
add_library(nav_distance_controller_bt_node SHARED behavior_tree/plugins/decorator/distance_controller.cc)
target_link_libraries(nav_distance_controller_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_distance_controller_bt_node)

# library(goal_updated_controller)
add_library(nav_goal_updated_controller_bt_node SHARED behavior_tree/plugins/decorator/goal_updated_controller.cc)
target_link_libraries(nav_goal_updated_controller_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_goal_updated_controller_bt_node)

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
