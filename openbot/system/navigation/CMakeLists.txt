# library()
set(library_name nav_behavior_tree)
add_library(${library_name} SHARED behavior_tree/behavior_tree_engine.cc)
target_link_libraries(${library_name} ${PROJECT_NAME})

###################################################################################################################
#
#         library action
#
###################################################################################################################

# library(assisted_teleop_action)
add_library(nav_assisted_teleop_action_bt_node SHARED behavior_tree/plugins/action/assisted_teleop_action.cc)
list(APPEND plugin_libs nav_assisted_teleop_action_bt_node)

# library(compute_path_through_poses_action)
add_library(nav_compute_path_through_poses_action_bt_node SHARED behavior_tree/plugins/action/compute_path_through_poses_action.cc)
list(APPEND plugin_libs nav_compute_path_through_poses_action_bt_node)

# library(compute_path_to_pose_action)
add_library(nav_compute_path_to_pose_action_bt_node SHARED behavior_tree/plugins/action/compute_path_to_pose_action.cc)
list(APPEND plugin_libs nav_compute_path_to_pose_action_bt_node)

# library(controller_selector)
add_library(nav_controller_selector_bt_node SHARED behavior_tree/plugins/action/controller_selector_node.cc)
list(APPEND plugin_libs nav_controller_selector_bt_node)

# library(follow_path_action)
add_library(nav_follow_path_action_bt_node SHARED behavior_tree/plugins/action/follow_path_action.cc)
list(APPEND plugin_libs nav_follow_path_action_bt_node)

# library(goal_checker_selector)
add_library(nav_goal_checker_selector_bt_node SHARED behavior_tree/plugins/action/goal_checker_selector_node.cc)
list(APPEND plugin_libs nav_goal_checker_selector_bt_node)

# library(planner_selector_node)
add_library(nav_planner_selector_node_bt_node SHARED behavior_tree/plugins/action/planner_selector_node.cc)
list(APPEND plugin_libs nav_planner_selector_node_bt_node)

# library(smooth_path_action)
add_library(nav_smooth_path_action_bt_node SHARED behavior_tree/plugins/action/smooth_path_action.cc)
list(APPEND plugin_libs nav_smooth_path_action_bt_node)

# library(spin_action)
add_library(nav_spin_action_bt_node SHARED behavior_tree/plugins/action/spin_action.cc)
list(APPEND plugin_libs nav_spin_action_bt_node)

# library(wait_action)
add_library(nav_wait_action_bt_node SHARED behavior_tree/plugins/action/wait_action.cc)
list(APPEND plugin_libs nav_wait_action_bt_node)


###################################################################################################################
#
#         library condition
#
###################################################################################################################

# library(distance_traveled_condition)
add_library(nav_distance_traveled_condition_bt_node SHARED behavior_tree/plugins/condition/distance_traveled_condition.cc)
list(APPEND plugin_libs nav_distance_traveled_condition_bt_node)

# library(globally_updated_goal_condition)
add_library(nav_globally_updated_goal_condition_bt_node SHARED behavior_tree/plugins/condition/globally_updated_goal_condition.cc)
list(APPEND plugin_libs nav_globally_updated_goal_condition_bt_node)

# library(goal_reached_condition)
add_library(nav_goal_reached_condition_bt_node SHARED behavior_tree/plugins/condition/goal_reached_condition.cc)
list(APPEND plugin_libs nav_goal_reached_condition_bt_node)

# library(goal_updated_condition)
add_library(nav_goal_updated_condition_bt_node SHARED behavior_tree/plugins/condition/goal_updated_condition.cc)
list(APPEND plugin_libs nav_goal_updated_condition_bt_node)

# library(initial_pose_received_condition)
add_library(nav_initial_pose_received_condition_bt_node SHARED behavior_tree/plugins/condition/initial_pose_received_condition.cc)
list(APPEND plugin_libs nav_initial_pose_received_condition_bt_node)

# library(is_battery_charging_condition)
add_library(nav_is_battery_charging_condition_bt_node SHARED behavior_tree/plugins/condition/is_battery_charging_condition.cc)
list(APPEND plugin_libs nav_is_battery_charging_condition_bt_node)

# library(is_battery_low_condition)
add_library(nav_is_battery_low_condition_bt_node SHARED behavior_tree/plugins/condition/is_battery_low_condition.cc)
list(APPEND plugin_libs nav_is_battery_low_condition_bt_node)

# library(is_path_valid_condition)
add_library(nav_is_path_valid_condition_bt_node SHARED behavior_tree/plugins/condition/is_path_valid_condition.cc)
list(APPEND plugin_libs nav_is_path_valid_condition_bt_node)

# library(is_stuck_condition)
add_library(nav_is_stuck_condition_bt_node SHARED behavior_tree/plugins/condition/is_stuck_condition.cc)
list(APPEND plugin_libs nav_is_stuck_condition_bt_node)

# library(path_expiring_timer_condition)
add_library(nav_path_expiring_timer_condition_bt_node SHARED behavior_tree/plugins/condition/path_expiring_timer_condition.cc)
list(APPEND plugin_libs nav_path_expiring_timer_condition_bt_node)

# library(time_expired_condition)
add_library(nav_time_expired_condition_bt_node SHARED behavior_tree/plugins/condition/time_expired_condition.cc)
list(APPEND plugin_libs nav_time_expired_condition_bt_node)

# library(transform_available_condition)
add_library(nav_transform_available_condition_bt_node SHARED behavior_tree/plugins/condition/transform_available_condition.cc)
list(APPEND plugin_libs nav_transform_available_condition_bt_node)


###################################################################################################################
#
#         library control
#
###################################################################################################################

# library(pipeline_sequence)
add_library(nav_pipeline_sequence_bt_node SHARED behavior_tree/plugins/control/pipeline_sequence.cc)
list(APPEND plugin_libs nav_pipeline_sequence_bt_node)

# library(recovery_node)
add_library(nav_recovery_node_bt_node SHARED behavior_tree/plugins/control/recovery_node.cc)
list(APPEND plugin_libs nav_recovery_node_bt_node)

# library(round_robin_node)
add_library(nav_round_robin_node_bt_node SHARED behavior_tree/plugins/control/round_robin_node.cc)
list(APPEND plugin_libs nav_round_robin_node_bt_node)

###################################################################################################################
#
#         library decorator
#
###################################################################################################################

# library(distance_controller)
add_library(nav_distance_controller_bt_node SHARED behavior_tree/plugins/decorator/distance_controller.cc)
list(APPEND plugin_libs nav_distance_controller_bt_node)

# library(goal_updated_controller)
add_library(nav_goal_updated_controller_bt_node SHARED behavior_tree/plugins/decorator/goal_updated_controller.cc)
list(APPEND plugin_libs nav_goal_updated_controller_bt_node)

# library(goal_updater_node)
add_library(nav_goal_updater_node_bt_node SHARED behavior_tree/plugins/decorator/goal_updater_node.cc)
list(APPEND plugin_libs nav_goal_updater_node_bt_node)

# library(path_longer_on_approach)
add_library(nav_path_longer_on_approach_bt_node SHARED behavior_tree/plugins/decorator/path_longer_on_approach.cc)
list(APPEND plugin_libs nav_path_longer_on_approach_bt_node)

# library(single_trigger_node)
add_library(nav_single_trigger_node_bt_node SHARED behavior_tree/plugins/decorator/single_trigger_node.cc)
list(APPEND plugin_libs nav_single_trigger_node_bt_node)

# library(rate_controller)
add_library(nav_rate_controller_bt_node SHARED behavior_tree/plugins/decorator/rate_controller.cc)
list(APPEND plugin_libs nav_rate_controller_bt_node)

# library(speed_controller)
add_library(nav_speed_controller_bt_node SHARED behavior_tree/plugins/decorator/speed_controller.cc)
list(APPEND plugin_libs nav_speed_controller_bt_node)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${PROJECT_NAME})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

###################################################################################################################
#
#         library compontents
#
###################################################################################################################

# bt_navigator_compontent
add_library(bt_navigator_compontent SHARED components/bt_navigator_compontent.cc)
target_link_libraries(bt_navigator_compontent ${PROJECT_NAME})
list(APPEND plugin_libs bt_navigator_compontent)

###################################################################################################################
#
#         For tests
#
###################################################################################################################

# openbot.test.navigate_to_pose_client_main
google_binary(openbot.test.navigate_to_pose_client_main
  SRCS
  naviagtor/navigate_to_pose_client_main.cc
)

# install
install(TARGETS 
  ${library_name}
  ${plugin_libs}
  openbot.test.navigate_to_pose_client_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
