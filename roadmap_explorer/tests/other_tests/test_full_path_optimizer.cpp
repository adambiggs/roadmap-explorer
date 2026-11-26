#include <gtest/gtest.h>

#include "roadmap_explorer/FullPathOptimizer.hpp"

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

class FullPathOptimizerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::InitOptions options;
    options.set_domain_id(91);
    rclcpp::init(0, nullptr, options);
    node_ = std::make_shared<nav2_util::LifecycleNode>("test_full_path_optimizer");
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_full_path_costmap");
    ASSERT_EQ(costmap_->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
    // A free corridor up column 1, with an isolated free cell at (6, 1).
    auto * grid = costmap_->getCostmap();
    grid->resizeMap(7, 7, 1.0, 0.0, 0.0);
    grid->resetMapToValue(0, 0, 7, 7, nav2_costmap_2d::LETHAL_OBSTACLE);
    for (unsigned int y = 1; y <= 6; ++y) {
      grid->setCost(1, y, nav2_costmap_2d::FREE_SPACE);
    }
    grid->setCost(6, 1, nav2_costmap_2d::FREE_SPACE);

    ParameterHandler::createInstance();
    parameterInstance.makeParameters(node_);
    parameterInstance.setValue("frontierRoadmap.radius_to_decide_edges", 11.0);
    RosVisualizer::createInstance(node_, grid);
    EventLogger::createInstance();
    FrontierRoadMap::createInstance(costmap_, node_);

    auto & graph = FrontierRoadMap::getInstance();
    graph.addNodes({start_, isolated_, reachable_}, false);
    graph.constructNewEdges({start_, isolated_, reachable_});
    optimizer_ = std::make_unique<FullPathOptimizer>(node_, costmap_);
  }

  void TearDown() override
  {
    optimizer_.reset();
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
  std::unique_ptr<FullPathOptimizer> optimizer_;
  FrontierPtr start_ = frontier(1.5, 1.5);
  FrontierPtr isolated_ = frontier(6.5, 1.5);
  FrontierPtr reachable_ = frontier(1.5, 5.5);
};

TEST_F(FullPathOptimizerTest, CachedUnreachableLegKeepsItsPenalty)
{
  ASSERT_FALSE(FrontierRoadMap::getInstance().getPlan(start_, true, isolated_, true).path_exists);

  const double fresh = optimizer_->calculateLengthRobotToGoal(start_, isolated_);
  EXPECT_GT(fresh, 0.0);
  EXPECT_DOUBLE_EQ(optimizer_->calculateLengthRobotToGoal(start_, isolated_), fresh);
  EXPECT_DOUBLE_EQ(optimizer_->calculateLengthRobotToGoal(isolated_, start_), fresh);
}

TEST_F(FullPathOptimizerTest, CachedReachableLegKeepsItsLength)
{
  ASSERT_TRUE(FrontierRoadMap::getInstance().getPlan(start_, true, reachable_, true).path_exists);

  EXPECT_DOUBLE_EQ(optimizer_->calculateLengthRobotToGoal(start_, reachable_), 4.0);
  EXPECT_DOUBLE_EQ(optimizer_->calculateLengthRobotToGoal(start_, reachable_), 4.0);
  EXPECT_DOUBLE_EQ(optimizer_->calculateLengthRobotToGoal(reachable_, start_), 4.0);
}

TEST_F(FullPathOptimizerTest, CachedZeroLengthLegCostsNothing)
{
  EXPECT_DOUBLE_EQ(optimizer_->calculateLengthRobotToGoal(start_, start_), 0.0);
  EXPECT_DOUBLE_EQ(optimizer_->calculateLengthRobotToGoal(start_, start_), 0.0);
}
}  // namespace
}  // namespace roadmap_explorer
