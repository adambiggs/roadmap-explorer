#include <gtest/gtest.h>

#include <nav2_costmap_2d/cost_values.hpp>

#include "roadmap_explorer/planners/OneTurnSearch.hpp"

namespace roadmap_explorer
{
namespace
{
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

constexpr double kSearchRadius = 11.0;

OneTurnSearch::Point cellCenter(unsigned int x, unsigned int y)
{
  OneTurnSearch::Point point;
  point.x = x + 0.5;
  point.y = y + 0.5;
  return point;
}

class OneTurnSearchTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // These one-cell-wide legs meet at (1, 6). Every other route hits lethal cells.
    for (unsigned int y = 1; y <= 6; ++y) {
      costmap_.setCost(1, y, FREE_SPACE);
    }
    for (unsigned int x = 1; x <= 6; ++x) {
      costmap_.setCost(x, 6, FREE_SPACE);
    }
  }

  nav2_costmap_2d::Costmap2D costmap_{7, 7, 1.0, 0.0, 0.0, LETHAL_OBSTACLE};
  const OneTurnSearch::Point start_ = cellCenter(1, 1);
  const OneTurnSearch::Point target_ = cellCenter(6, 6);
  bool canceled_{false};
  const std::function<bool()> canceled = [this]() {return canceled_;};
};

TEST_F(OneTurnSearchTest, FindsTurnWhenStraightConnectionCrossesObstacles)
{
  ASSERT_EQ(costmap_.getCost(3, 3), LETHAL_OBSTACLE);
  OneTurnSearch search(costmap_, start_, kSearchRadius, canceled);
  const auto length = search.connectionLength(start_, target_, target_);
  ASSERT_TRUE(length.has_value());
  EXPECT_DOUBLE_EQ(*length, 10.0);
}

TEST_F(OneTurnSearchTest, RejectsSolidBarrierAndInscribedCell)
{
  OneTurnSearch open(costmap_, start_, kSearchRadius, canceled);
  ASSERT_TRUE(open.connectionLength(start_, target_, target_).has_value());

  for (const auto obstacle : {LETHAL_OBSTACLE, INSCRIBED_INFLATED_OBSTACLE}) {
    costmap_.setCost(1, 4, obstacle);
    OneTurnSearch blocked(costmap_, start_, kSearchRadius, canceled);
    EXPECT_FALSE(blocked.connectionLength(start_, target_, target_).has_value())
      << "cost " << static_cast<int>(obstacle);
  }
}

TEST_F(OneTurnSearchTest, BothSegmentsSpendTheSameUnknownBudget)
{
  costmap_.setCost(1, 2, NO_INFORMATION);
  costmap_.setCost(1, 3, NO_INFORMATION);
  costmap_.setCost(4, 6, NO_INFORMATION);
  costmap_.setCost(5, 6, NO_INFORMATION);
  // Known cells offer a grid route around each unknown pair, but reaching the
  // goal that way takes extra turns. The one-turn rays still spend four cells.
  for (unsigned int y = 1; y <= 4; ++y) {
    costmap_.setCost(0, y, FREE_SPACE);
  }
  for (unsigned int x = 3; x <= 6; ++x) {
    costmap_.setCost(x, 5, FREE_SPACE);
  }

  // Each leg spends two cells. For a 1 m resolution, the 30% allowance is
  // 4.2 cells at 14 m and 3.3 cells at 11 m. Both radii cover the 10 m route.
  OneTurnSearch enough(costmap_, start_, 14.0, canceled);
  const auto length = enough.connectionLength(start_, target_, target_);
  ASSERT_TRUE(length.has_value());
  EXPECT_DOUBLE_EQ(*length, 10.0);

  OneTurnSearch over_budget(costmap_, start_, kSearchRadius, canceled);
  EXPECT_FALSE(over_budget.connectionLength(start_, target_, target_).has_value());

  costmap_.setCost(5, 6, FREE_SPACE);
  OneTurnSearch under_budget(costmap_, start_, kSearchRadius, canceled);
  EXPECT_TRUE(under_budget.connectionLength(start_, target_, target_).has_value());
}

TEST_F(OneTurnSearchTest, AcceptsMapRimAndRejectsPointsOutsideTheGrid)
{
  OneTurnSearch search(costmap_, start_, kSearchRadius, canceled);
  ASSERT_TRUE(search.connectionLength(start_, target_, target_).has_value());

  auto outside = target_;
  outside.y = 7.0;
  EXPECT_FALSE(search.connectionLength(start_, target_, outside).has_value());
  EXPECT_FALSE(search.connectionLength(start_, outside, target_).has_value());
  EXPECT_FALSE(search.connectionLength(outside, target_, target_).has_value());
}

TEST_F(OneTurnSearchTest, StopsSearchingWhenCanceled)
{
  OneTurnSearch search(costmap_, start_, kSearchRadius, canceled);
  ASSERT_TRUE(search.connectionLength(start_, target_, target_).has_value());

  canceled_ = true;
  EXPECT_FALSE(search.connectionLength(start_, target_, target_).has_value());
  OneTurnSearch already_canceled(costmap_, start_, kSearchRadius, canceled);
  EXPECT_FALSE(already_canceled.connectionLength(start_, target_, target_).has_value());
}

TEST_F(OneTurnSearchTest, StillConnectsAReachableTargetAfterAnUnreachableOne)
{
  costmap_.setCost(6, 1, FREE_SPACE);
  const auto isolated = cellCenter(6, 1);
  OneTurnSearch search(costmap_, start_, kSearchRadius, canceled);
  EXPECT_FALSE(search.connectionLength(start_, isolated, isolated).has_value());

  const auto length = search.connectionLength(start_, target_, target_);
  ASSERT_TRUE(length.has_value());
  EXPECT_DOUBLE_EQ(*length, 10.0);
}

TEST(OneTurnSearch, FindsClearTurnBeyondTheShortestGridPath)
{
  nav2_costmap_2d::Costmap2D costmap(8, 8, 1.0, 0.0, 0.0, FREE_SPACE);
  for (unsigned int y = 0; y <= 3; ++y) {
    costmap.setCost(2, y, LETHAL_OBSTACLE);
  }
  const auto start = cellCenter(1, 1);
  const auto end = cellCenter(4, 1);

  // A turn at cell (1, 6) sees both endpoints around the wall. Following
  // only the shortest four-neighbor path misses every feasible turning cell.
  OneTurnSearch search(costmap, start, 16.0, []() {return false;});
  EXPECT_TRUE(search.connectionLength(start, end, end).has_value());
}

TEST(OneTurnSearch, KeepsAnObliqueTurnJustInsideTheRadius)
{
  nav2_costmap_2d::Costmap2D costmap(6, 6, 1.0, 0.0, 0.0, FREE_SPACE);
  for (unsigned int y = 0; y <= 2; ++y) {
    costmap.setCost(2, y, LETHAL_OBSTACLE);
  }
  auto robot = cellCenter(1, 1);
  robot.x = robot.y = 1.95;
  auto goal = cellCenter(4, 2);
  goal.x = 4.05;
  goal.y = 2.95;
  auto snapped_start = robot;
  snapped_start.x = snapped_start.y = 1.94;
  auto snapped_end = goal;
  snapped_end.x = 4.06;
  snapped_end.y = 2.94;

  // The bend at (1.5, 3.5) gives a 4.223 m physical route, even though
  // the shortest grid-center octile path is about 4.83 m.
  constexpr double radius = 4.3;
  OneTurnSearch search(costmap, robot, radius, []() {return false;});
  const auto length = search.connectionLength(snapped_start, snapped_end, goal);
  ASSERT_TRUE(length.has_value());
  EXPECT_GT(*length, 4.2);
  EXPECT_LT(*length, radius);
}

TEST(OneTurnSearch, ChecksTheOriginalMapCellAtACroppedBoundary)
{
  nav2_costmap_2d::Costmap2D costmap(500, 500, 0.05, 0.0, 0.0, FREE_SPACE);
  OneTurnSearch::Point robot;
  robot.x = 10.123;
  robot.y = 1.025;
  OneTurnSearch::Point goal;
  goal.x = 0.25;
  goal.y = robot.y;
  unsigned int x, y;
  ASSERT_TRUE(costmap.worldToMap(goal.x, goal.y, x, y));
  ASSERT_EQ(x, 5u);
  ASSERT_EQ(y, 20u);

  // Recomputing the cell relative to the cropped origin can place this
  // exact boundary point in the free cell beside the lethal source cell.
  costmap.setCost(x, y, LETHAL_OBSTACLE);
  OneTurnSearch blocked(costmap, robot, 10.0, []() {return false;});
  EXPECT_FALSE(blocked.connectionLength(robot, goal, goal).has_value());

  costmap.setCost(x, y, FREE_SPACE);
  OneTurnSearch open(costmap, robot, 10.0, []() {return false;});
  EXPECT_TRUE(open.connectionLength(robot, goal, goal).has_value());
}
}  // namespace
}  // namespace roadmap_explorer
