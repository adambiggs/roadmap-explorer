#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "roadmap_explorer/CostAssigner.hpp"
#include "roadmap_explorer/FullPathOptimizer.hpp"

namespace roadmap_explorer
{
namespace
{
using namespace std::chrono_literals;

FrontierPtr frontier(double x, double y, double path_length = 0.0)
{
  auto f = std::make_shared<Frontier>();
  f->setGoalPoint(x, y);
  f->setUID(generateUID(f));
  f->setAchievability(true);
  f->setPathLength(path_length);
  f->setPathLengthInM(path_length);
  return f;
}

// Two frontiers on either side of the robot, re-detected on every search as
// the explorer does: new objects whose goal points move a few centimetres
// and whose path lengths take turns being the shorter one.
class StickyExplorationGoalTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::InitOptions options;
    options.set_domain_id(92);
    rclcpp::init(0, nullptr, options);
    node_ = std::make_shared<nav2_util::LifecycleNode>("test_sticky_goal");
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_sticky_goal_costmap");
    ASSERT_EQ(costmap_->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
    auto * grid = costmap_->getCostmap();
    grid->resizeMap(20, 20, 1.0, 0.0, 0.0);
    grid->resetMapToValue(0, 0, 20, 20, nav2_costmap_2d::FREE_SPACE);

    ParameterHandler::createInstance();
    parameterInstance.makeParameters(node_);
    RosVisualizer::createInstance(node_, grid);
    EventLogger::createInstance();
    FrontierRoadMap::createInstance(costmap_, node_);
    auto & graph = FrontierRoadMap::getInstance();
    std::vector<FrontierPtr> nodes{frontier(10.0, 10.0), frontier(13.0, 10.0), frontier(7.0, 10.0)};
    graph.addNodes(nodes, false);
    graph.constructNewEdges(nodes);

    optimizer_ = std::make_shared<FullPathOptimizer>(node_, costmap_);
    robot_.header.frame_id = "map";
    robot_.pose.position.x = 10.0;
    robot_.pose.position.y = 10.0;
    robot_.pose.orientation.w = 1.0;
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

  // One search: east frontier and west frontier, the shorter one alternating.
  std::vector<FrontierPtr> search(int cycle)
  {
    const double jitter = 0.05 * (cycle % 4);
    const bool east_shorter = cycle % 2 == 0;
    return {frontier(13.0 + jitter, 10.0, east_shorter ? 3.0 : 3.3),
      frontier(7.0, 10.0 + jitter, east_shorter ? 3.3 : 3.0)};
  }

  FrontierPtr next(std::vector<FrontierPtr> frontiers, FrontierPtr * abandoned = nullptr)
  {
    FrontierPtr goal;
    FrontierPtr given_up;
    EXPECT_TRUE(optimizer_->getNextGoal(frontiers, goal, given_up, robot_, true));
    if (abandoned) {
      *abandoned = given_up;
    } else {
      EXPECT_FALSE(given_up);
    }
    return goal;
  }

  static bool east(const FrontierPtr & goal) {return goal->getGoalPoint().x > 10.0;}

  std::shared_ptr<nav2_util::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<FullPathOptimizer> optimizer_;
  geometry_msgs::msg::PoseStamped robot_;
};

TEST_F(StickyExplorationGoalTest, KeepsOneGoalWhileTwoFrontiersTradeTheLead)
{
  ASSERT_TRUE(east(next(search(0))));
  for (int cycle = 1; cycle < 12; ++cycle) {
    EXPECT_TRUE(east(next(search(cycle)))) << "switched frontiers on search " << cycle;
  }
}

TEST_F(StickyExplorationGoalTest, ChoosesANewGoalWhenTheCommittedFrontierIsGone)
{
  ASSERT_TRUE(east(next(search(0))));
  auto explored = search(1);
  explored.front()->setAchievability(false);
  EXPECT_FALSE(east(next(explored)));
  // The new goal is now the sticky one.
  EXPECT_FALSE(east(next(search(2))));
}

TEST_F(StickyExplorationGoalTest, SetsAsideAGoalWithNoProgressAndRetriesItAfterTheCooldown)
{
  parameterInstance.setValue("fullPathOptimizer.goal_stuck_timeout", 0.2);
  parameterInstance.setValue("fullPathOptimizer.goal_retry_cooldown", 0.5);
  ASSERT_TRUE(east(next(search(0))));
  EXPECT_TRUE(east(next(search(1))));

  std::this_thread::sleep_for(250ms);
  const auto frontiers = search(2);
  EXPECT_FALSE(east(next(frontiers)));
  EXPECT_TRUE(frontiers.front()->isBlacklisted());
  // Still cooling down: the west goal is kept while it lasts.
  EXPECT_FALSE(east(next(search(3))));

  // Once the cooldown ends the east frontier competes again, and beats a
  // third one when the west goal is gone.
  std::this_thread::sleep_for(600ms);
  auto explored = search(4);
  explored.back()->setAchievability(false);
  explored.push_back(frontier(10.0, 13.0, 3.6));
  EXPECT_TRUE(east(next(explored)));
}

TEST_F(StickyExplorationGoalTest, KeepsAnEarlyRetryWhenAnotherFrontierAppears)
{
  parameterInstance.setValue("fullPathOptimizer.goal_stuck_timeout", 0.2);
  parameterInstance.setValue("fullPathOptimizer.goal_retry_cooldown", 60.0);
  const auto eastOnly = [this](int cycle) {
      auto frontiers = search(cycle);
      frontiers.back()->setAchievability(false);
      return frontiers;
    };
  ASSERT_TRUE(east(next(eastOnly(0))));

  // Set aside with nothing else left, so it is retried at once.
  std::this_thread::sleep_for(250ms);
  ASSERT_TRUE(east(next(eastOnly(1))));
  // The retry stays the goal although west appears during the cooldown.
  EXPECT_TRUE(east(next(search(3))));
}

TEST_F(StickyExplorationGoalTest, BlacklistsAGoalOnItsLastAttempt)
{
  parameterInstance.setValue("fullPathOptimizer.goal_stuck_timeout", 0.2);
  parameterInstance.setValue("fullPathOptimizer.goal_max_attempts", int64_t{1});
  ASSERT_TRUE(east(next(search(0))));

  std::this_thread::sleep_for(250ms);
  const auto frontiers = search(1);
  FrontierPtr abandoned;
  EXPECT_FALSE(east(next(frontiers, &abandoned)));
  ASSERT_TRUE(abandoned);
  EXPECT_TRUE(east(abandoned));
  EXPECT_TRUE(frontiers.front()->isBlacklisted());
}

TEST_F(StickyExplorationGoalTest, BlacklistCoversTheFrontierAfterItsPointMoves)
{
  node_->declare_parameter(
    "costAssigner.FrontierRoadmap.plugin_type",
    std::string("roadmap_explorer::EuclideanDistancePlugin"));
  node_->declare_parameter("costAssigner.CountBasedGain.factor_of_max_is_min", 0.0);
  // Unknown space beyond both frontiers gives them information to gain.
  auto * grid = costmap_->getCostmap();
  for (unsigned int y = 0; y < 20; ++y) {
    for (unsigned int x : {0u, 1u, 2u, 3u, 4u, 5u, 15u, 16u, 17u, 18u, 19u}) {
      grid->setCost(x, y, nav2_costmap_2d::NO_INFORMATION);
    }
  }
  CostAssigner costs(costmap_, node_);
  geometry_msgs::msg::PolygonStamped everywhere;
  costs.updateBoundaryPolygon(everywhere);

  auto request = std::make_shared<CalculateFrontierCostsRequest>();
  request->start_pose = robot_;
  auto moved = frontier(13.3, 10.2);
  auto other = frontier(7.0, 10.0);
  request->frontier_list = {moved, other};
  request->prohibited_frontiers = {frontier(13.0, 10.0)};
  ASSERT_TRUE(costs.populateFrontierCosts(request));
  EXPECT_FALSE(moved->isAchievable());
  EXPECT_TRUE(other->isAchievable());
}
}  // namespace
}  // namespace roadmap_explorer
