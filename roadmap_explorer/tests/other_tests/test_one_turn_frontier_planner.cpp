#include <gtest/gtest.h>

#include "roadmap_explorer/planners/FrontierRoadmapPlugin.hpp"

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

class OneTurnFrontierPlannerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::InitOptions options;
    options.set_domain_id(91);
    rclcpp::init(0, nullptr, options);
    node_ = std::make_shared<nav2_util::LifecycleNode>("test_frontier_planner");
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_frontier_costmap");
    ASSERT_EQ(costmap_->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
    auto * grid = costmap_->getCostmap();
    grid->resizeMap(7, 7, 1.0, 0.0, 0.0);
    grid->resetMapToValue(0, 0, 7, 7, nav2_costmap_2d::LETHAL_OBSTACLE);
    for (unsigned int y = 1; y <= 6; ++y) {
      grid->setCost(1, y, nav2_costmap_2d::FREE_SPACE);
    }
    for (unsigned int x = 1; x <= 6; ++x) {
      grid->setCost(x, 6, nav2_costmap_2d::FREE_SPACE);
    }
    grid->setCost(6, 1, nav2_costmap_2d::FREE_SPACE);

    ParameterHandler::createInstance();
    parameterInstance.makeParameters(node_);
    parameterInstance.setValue("frontierRoadmap.radius_to_decide_edges", 11.0);
    RosVisualizer::createInstance(node_, grid);
    EventLogger::createInstance();
    FrontierRoadMap::createInstance(costmap_, node_);

    auto & graph = FrontierRoadMap::getInstance();
    graph.addNodes({start_node_, impossible_, around_corner_}, false);
    graph.constructNewEdges({start_node_, impossible_, around_corner_});
    start_.position = start_node_->getGoalPoint();
    node_->declare_parameter("planner.max_planning_distance_roadmap", 11.0);
    planner_.configure(costmap_, "planner", node_);
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
  PluginFrontierRoadmap planner_;
  geometry_msgs::msg::Pose start_;
  FrontierPtr start_node_ = frontier(1.5, 1.5);
  FrontierPtr impossible_ = frontier(6.5, 1.5);
  FrontierPtr around_corner_ = frontier(6.5, 6.5);
};

TEST_F(OneTurnFrontierPlannerTest, RecoversFartherCandidateAfterCloserCandidateFails)
{
  auto & graph = FrontierRoadMap::getInstance();
  ASSERT_FALSE(graph.getPlan(start_node_, true, impossible_, true).path_exists);
  ASSERT_FALSE(graph.getPlan(start_node_, true, around_corner_, true).path_exists);

  std::vector<FrontierPtr> frontiers{impossible_, around_corner_};
  EXPECT_TRUE(planner_.setPlansForFrontiers(start_, frontiers, []() {return false;}));
  EXPECT_FALSE(impossible_->isAchievable());
  ASSERT_TRUE(around_corner_->isAchievable());
  EXPECT_DOUBLE_EQ(around_corner_->getPathLengthInM(), 10.0);
}

TEST_F(OneTurnFrontierPlannerTest, KeepsOrdinaryRoadmapPlanWhenOneExists)
{
  auto & graph = FrontierRoadMap::getInstance();
  auto direct = frontier(1.5, 5.5);
  graph.addNodes({direct}, false);
  graph.constructNewEdges({direct});
  ASSERT_TRUE(graph.getPlan(start_node_, true, direct, true).path_exists);
  ASSERT_FALSE(graph.getPlan(start_node_, true, around_corner_, true).path_exists);

  std::vector<FrontierPtr> frontiers{around_corner_, direct};
  EXPECT_FALSE(planner_.setPlansForFrontiers(start_, frontiers, []() {return false;}));
  EXPECT_FALSE(around_corner_->isAchievable());
  ASSERT_TRUE(direct->isAchievable());
  EXPECT_DOUBLE_EQ(direct->getPathLengthInM(), 4.0);
}

TEST_F(OneTurnFrontierPlannerTest, ScoresEveryRecoverableFrontierAtItsOriginalGoal)
{
  auto & graph = FrontierRoadMap::getInstance();
  auto nearer = frontier(4.5, 6.5);
  graph.addNodes({nearer}, false);
  graph.constructNewEdges({nearer});
  ASSERT_FALSE(graph.getPlan(start_node_, true, nearer, true).path_exists);
  ASSERT_FALSE(graph.getPlan(start_node_, true, around_corner_, true).path_exists);
  const auto nearer_goal = nearer->getGoalPoint();
  const auto farther_goal = around_corner_->getGoalPoint();

  std::vector<FrontierPtr> frontiers{around_corner_, nearer};
  EXPECT_TRUE(planner_.setPlansForFrontiers(start_, frontiers, []() {return false;}));
  ASSERT_TRUE(nearer->isAchievable());
  ASSERT_TRUE(around_corner_->isAchievable());
  EXPECT_DOUBLE_EQ(nearer->getPathLengthInM(), 8.0);
  EXPECT_DOUBLE_EQ(around_corner_->getPathLengthInM(), 10.0);
  EXPECT_EQ(nearer->getGoalPoint(), nearer_goal);
  EXPECT_EQ(around_corner_->getGoalPoint(), farther_goal);
}

TEST_F(OneTurnFrontierPlannerTest, CountsFailedRoadmapPlansUntilTaken)
{
  auto & graph = FrontierRoadMap::getInstance();
  graph.takePlanFailures();
  ASSERT_FALSE(graph.getPlan(start_node_, true, impossible_, true).path_exists);
  ASSERT_FALSE(graph.getPlan(start_node_, true, around_corner_, true).path_exists);

  auto failures = graph.takePlanFailures();
  EXPECT_EQ(failures.count, 2u);
  EXPECT_DOUBLE_EQ(failures.first_start.x, 1.5);
  EXPECT_DOUBLE_EQ(failures.first_start.y, 1.5);
  EXPECT_DOUBLE_EQ(failures.first_target.x, 6.5);
  EXPECT_DOUBLE_EQ(failures.first_target.y, 1.5);
  EXPECT_EQ(graph.takePlanFailures().count, 0u);
}
}  // namespace
}  // namespace roadmap_explorer
