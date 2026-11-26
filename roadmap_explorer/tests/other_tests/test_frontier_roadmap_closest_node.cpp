#include <gtest/gtest.h>

#include "roadmap_explorer/planners/FrontierRoadmap.hpp"

namespace roadmap_explorer
{
namespace
{
FrontierPtr frontier(double x, double y)
{
  auto node = std::make_shared<Frontier>();
  node->setGoalPoint(x, y);
  node->setUID(generateUID(node));
  return node;
}

class FrontierRoadmapClosestNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::InitOptions options;
    options.set_domain_id(91);
    rclcpp::init(0, nullptr, options);
    node_ = std::make_shared<nav2_util::LifecycleNode>("test_roadmap_closest_node");
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_roadmap_closest_costmap");
    ASSERT_EQ(costmap_->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
    // Free space from x = -2 m to 3 m, with a lethal wall at 0.2 <= x < 0.3.
    auto * grid = costmap_->getCostmap();
    grid->resizeMap(100, 40, 0.05, -2.0, -1.0);
    grid->resetMapToValue(0, 0, 100, 40, nav2_costmap_2d::FREE_SPACE);
    grid->resetMapToValue(44, 0, 46, 40, nav2_costmap_2d::LETHAL_OBSTACLE);

    ParameterHandler::createInstance();
    parameterInstance.makeParameters(node_);
    parameterInstance.setValue("frontierRoadmap.grid_cell_size", 0.5);
    parameterInstance.setValue("frontierRoadmap.radius_to_decide_edges", 11.0);
    RosVisualizer::createInstance(node_, grid);
    EventLogger::createInstance();
    FrontierRoadMap::createInstance(costmap_, node_);
  }

  void TearDown() override
  {
    FrontierRoadMap::destroyInstance();
    EventLogger::destroyInstance();
    RosVisualizer::destroyInstance();
    ParameterHandler::destroyInstance();
    costmap_->on_cleanup(rclcpp_lifecycle::State());
    costmap_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<nav2_util::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
};

TEST_F(FrontierRoadmapClosestNodeTest, PlanStartsFromClosestNodeAcrossCellBoundary)
{
  // The closer node is in hash cell (-1, 0); the farther node shares the query's cell (0, 0)
  // but sits behind the wall.
  auto closer = frontier(-0.05, 0.02);
  auto farther = frontier(0.475, 0.45);
  auto target = frontier(-1.2, 0.02);
  auto & graph = FrontierRoadMap::getInstance();
  graph.addNodes({closer, farther, target}, false);
  graph.constructNewEdges({closer, farther, target});
  ASSERT_FALSE(graph.getPlan(farther, true, target, true).path_exists);

  auto plan = graph.getPlan(0.0116271, 0.0194011, true, -1.2, 0.02, true, false);

  ASSERT_TRUE(plan.path_exists);
  ASSERT_FALSE(plan.path.empty());
  EXPECT_EQ(plan.path.front()->frontier, closer);
}

TEST_F(FrontierRoadmapClosestNodeTest, PlanStartsFromAdjacentCellNodeWhenQueryIsOnCellEdge)
{
  // Hash cells are 0.5 m. The query sits just inside the right edge of cell (0, 0). A node
  // 0.45 m away shares its cell behind the wall; a node 0.025 m away is in cell (1, 0).
  auto same_cell = frontier(0.04, 0.25);
  auto next_cell = frontier(0.515, 0.25);
  auto target = frontier(1.2, 0.25);
  auto & graph = FrontierRoadMap::getInstance();
  graph.addNodes({same_cell, next_cell, target}, false);
  graph.constructNewEdges({same_cell, next_cell, target});
  ASSERT_FALSE(graph.getPlan(same_cell, true, target, true).path_exists);

  auto plan = graph.getPlan(0.49, 0.25, true, 1.2, 0.25, true, false);

  ASSERT_TRUE(plan.path_exists);
  ASSERT_FALSE(plan.path.empty());
  EXPECT_EQ(plan.path.front()->frontier, next_cell);
}
}  // namespace
}  // namespace roadmap_explorer
