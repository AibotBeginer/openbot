# library()
set(library_name nav_behavior_tree)
add_library(${library_name} SHARED behavior_tree/behavior_tree_engine.cc)
target_link_libraries(${library_name} ${PROJECT_NAME})

###################################################################################################################
#
#         library action
#
###################################################################################################################

# library(follow_path_action)
add_library(nav_follow_path_action_bt_node SHARED behavior_tree/plugins/action/follow_path_action.cc)
target_link_libraries(nav_follow_path_action_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_follow_path_action_bt_node)

# library(wait_action)
add_library(nav_wait_action_bt_node SHARED behavior_tree/plugins/action/wait_action.cc)
target_link_libraries(nav_wait_action_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_wait_action_bt_node)


###################################################################################################################
#
#         library condition
#
###################################################################################################################

# library(distance_traveled_condition)
add_library(nav_distance_traveled_condition_bt_node SHARED behavior_tree/plugins/condition/distance_traveled_condition.cc)
target_link_libraries(nav_distance_traveled_condition_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_distance_traveled_condition_bt_node)

# library(globally_updated_goal_condition)
add_library(nav_globally_updated_goal_condition_bt_node SHARED behavior_tree/plugins/condition/globally_updated_goal_condition.cc)
target_link_libraries(nav_globally_updated_goal_condition_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_globally_updated_goal_condition_bt_node)

# library(goal_reached_condition)
add_library(nav_goal_reached_condition_bt_node SHARED behavior_tree/plugins/condition/goal_reached_condition.cc)
target_link_libraries(nav_goal_reached_condition_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_goal_reached_condition_bt_node)

# library(goal_updated_condition)
add_library(nav_goal_updated_condition_bt_node SHARED behavior_tree/plugins/condition/goal_updated_condition.cc)
target_link_libraries(nav_goal_updated_condition_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_goal_updated_condition_bt_node)

# library(initial_pose_received_condition)
add_library(nav_initial_pose_received_condition_bt_node SHARED behavior_tree/plugins/condition/initial_pose_received_condition.cc)
target_link_libraries(nav_initial_pose_received_condition_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_initial_pose_received_condition_bt_node)

# library(is_battery_charging_condition)
add_library(nav_is_battery_charging_condition_bt_node SHARED behavior_tree/plugins/condition/is_battery_charging_condition.cc)
target_link_libraries(nav_is_battery_charging_condition_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_is_battery_charging_condition_bt_node)

# library(is_battery_low_condition)
add_library(nav_is_battery_low_condition_bt_node SHARED behavior_tree/plugins/condition/is_battery_low_condition.cc)
target_link_libraries(nav_is_battery_low_condition_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_is_battery_low_condition_bt_node)

# library(is_path_valid_condition)
add_library(nav_is_path_valid_condition_bt_node SHARED behavior_tree/plugins/condition/is_path_valid_condition.cc)
target_link_libraries(nav_is_path_valid_condition_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_is_path_valid_condition_bt_node)

# library(is_stuck_condition)
add_library(nav_is_stuck_condition_bt_node SHARED behavior_tree/plugins/condition/is_stuck_condition.cc)
target_link_libraries(nav_is_stuck_condition_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_is_stuck_condition_bt_node)

# library(path_expiring_timer_condition)
add_library(nav_path_expiring_timer_condition_bt_node SHARED behavior_tree/plugins/condition/path_expiring_timer_condition.cc)
target_link_libraries(nav_path_expiring_timer_condition_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_path_expiring_timer_condition_bt_node)

# library(time_expired_condition)
add_library(nav_time_expired_condition_bt_node SHARED behavior_tree/plugins/condition/time_expired_condition.cc)
target_link_libraries(nav_time_expired_condition_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_time_expired_condition_bt_node)

# library(transform_available_condition)
add_library(nav_transform_available_condition_bt_node SHARED behavior_tree/plugins/condition/transform_available_condition.cc)
target_link_libraries(nav_transform_available_condition_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_transform_available_condition_bt_node)


###################################################################################################################
#
#         library control
#
###################################################################################################################

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

###################################################################################################################
#
#         library decorator
#
###################################################################################################################

# library(distance_controller)
add_library(nav_distance_controller_bt_node SHARED behavior_tree/plugins/decorator/distance_controller.cc)
target_link_libraries(nav_distance_controller_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_distance_controller_bt_node)

# library(goal_updated_controller)
add_library(nav_goal_updated_controller_bt_node SHARED behavior_tree/plugins/decorator/goal_updated_controller.cc)
target_link_libraries(nav_goal_updated_controller_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_goal_updated_controller_bt_node)

# library(goal_updater_node)
add_library(nav_goal_updater_node_bt_node SHARED behavior_tree/plugins/decorator/goal_updater_node.cc)
target_link_libraries(nav_goal_updater_node_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_goal_updater_node_bt_node)

# library(path_longer_on_approach)
add_library(nav_path_longer_on_approach_bt_node SHARED behavior_tree/plugins/decorator/path_longer_on_approach.cc)
target_link_libraries(nav_path_longer_on_approach_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_path_longer_on_approach_bt_node)

# library(single_trigger_node)
add_library(nav_single_trigger_node_bt_node SHARED behavior_tree/plugins/decorator/single_trigger_node.cc)
target_link_libraries(nav_single_trigger_node_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_single_trigger_node_bt_node)

# library(rate_controller)
add_library(nav_rate_controller_bt_node SHARED behavior_tree/plugins/decorator/rate_controller.cc)
target_link_libraries(nav_rate_controller_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_rate_controller_bt_node)

# library(speed_controller)
add_library(nav_speed_controller_bt_node SHARED behavior_tree/plugins/decorator/speed_controller.cc)
target_link_libraries(nav_speed_controller_bt_node ${PROJECT_NAME})
list(APPEND plugin_libs nav_speed_controller_bt_node)

# openbot.application.navigation_main
google_binary(openbot.application.navigation_main
  SRCS
    navigation_main.cc
)

# install
install(TARGETS 
  ${library_name}
  ${plugin_libs}
  openbot.application.navigation_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
