cmake_minimum_required(VERSION 3.5)

# Find GTest
find_package(ament_cmake_gtest REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  nav2_util
  Eigen3
  nav2_costmap_2d
  sensor_msgs
  visualization_msgs
  nav_msgs
  pcl_ros
  PCL
  pluginlib
)

# Include directories
include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Test executable for CostAssigner
ament_add_gtest(test_cost_assigner test_cost_assigner.cpp)
ament_target_dependencies(test_cost_assigner 
  ${dependencies}
)
target_link_libraries(test_cost_assigner
  roadmap_explorer_core
  roadmap_explorer_frontier
  roadmap_explorer_utils
  roadmap_explorer_bt_plugins
  roadmap_explorer_frontier_search_plugins
  roadmap_explorer_planner_plugins
  roadmap_explorer_information_gain_plugins
)

# Set C++ standard for all tests
set_target_properties(test_cost_assigner PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
)


ament_add_gtest(test_navigation_cancel test_navigation_cancel.cpp)
ament_target_dependencies(test_navigation_cancel rclcpp_action nav2_msgs nav2_util tf2_ros)
target_link_libraries(test_navigation_cancel roadmap_explorer_core)

ament_add_gtest(test_one_turn_search test_one_turn_search.cpp ../../src/planners/OneTurnSearch.cpp)
ament_target_dependencies(test_one_turn_search geometry_msgs nav2_costmap_2d)

ament_add_gtest(test_one_turn_frontier_planner test_one_turn_frontier_planner.cpp)
ament_target_dependencies(test_one_turn_frontier_planner ${dependencies} roadmap_explorer_msgs)
target_link_libraries(test_one_turn_frontier_planner roadmap_explorer_core)

ament_add_gtest(test_full_path_optimizer test_full_path_optimizer.cpp)
ament_target_dependencies(test_full_path_optimizer ${dependencies} roadmap_explorer_msgs)
target_link_libraries(test_full_path_optimizer roadmap_explorer_core)

ament_add_gtest(test_frontier_roadmap_closest_node test_frontier_roadmap_closest_node.cpp)
ament_target_dependencies(test_frontier_roadmap_closest_node ${dependencies} roadmap_explorer_msgs)
target_link_libraries(test_frontier_roadmap_closest_node roadmap_explorer_core)
