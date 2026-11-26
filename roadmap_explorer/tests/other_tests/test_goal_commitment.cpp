#include <gtest/gtest.h>

#include <limits>
#include <map>
#include <memory>
#include <vector>

#include "roadmap_explorer/GoalCommitment.hpp"

namespace roadmap_explorer
{
namespace
{
using Outcome = GoalCommitment::Outcome;

constexpr GoalCommitment::Settings kSettings{1.0, 30.0, 0.25};

FrontierPtr frontier(double x, double y)
{
  auto f = std::make_shared<Frontier>();
  f->setGoalPoint(x, y);
  f->setUID(generateUID(f));
  f->setAchievability(true);
  return f;
}

class GoalCommitmentTest : public ::testing::Test
{
protected:
  GoalCommitment::Decision keep(const std::vector<FrontierPtr> & frontiers, double now)
  {
    return commitment_.keep(
      frontiers, [this](const FrontierPtr & f) {return remaining_.at(f);}, now, kSettings);
  }

  GoalCommitment commitment_;
  std::map<FrontierPtr, double> remaining_;
};

TEST_F(GoalCommitmentTest, NothingIsKeptBeforeACommitment)
{
  EXPECT_EQ(keep({frontier(1.0, 0.0)}, 0.0).outcome, Outcome::None);
}

TEST_F(GoalCommitmentTest, FollowsTheCommittedFrontierAsItsPointMoves)
{
  auto first = frontier(3.0, 0.0);
  commitment_.commit(first, 3.0, 0.0);

  auto moved = frontier(3.4, 0.3);
  auto other = frontier(-3.0, 0.0);
  remaining_ = {{moved, 3.4}, {other, 2.0}};
  const auto decision = keep({other, moved}, 1.0);
  EXPECT_EQ(decision.outcome, Outcome::Kept);
  EXPECT_EQ(decision.frontier, moved);
}

TEST_F(GoalCommitmentTest, KeepsTheNearestMatchToTheFirstCommittedPoint)
{
  commitment_.commit(frontier(3.0, 0.0), 3.0, 0.0);
  auto near = frontier(3.2, 0.0);
  auto edge = frontier(3.0, 0.9);
  remaining_ = {{near, 3.2}, {edge, 3.1}};
  EXPECT_EQ(keep({edge, near}, 1.0).frontier, near);

  // A later drift of the match does not move the committed point.
  auto drifted = frontier(3.9, 0.0);
  auto beyond = frontier(4.1, 0.0);
  remaining_ = {{drifted, 3.9}, {beyond, 4.1}};
  EXPECT_EQ(keep({drifted}, 2.0).frontier, drifted);
  EXPECT_EQ(keep({beyond}, 3.0).outcome, Outcome::Lost);
}

TEST_F(GoalCommitmentTest, LosesTheGoalWhenNoAchievableFrontierIsLeftAtIt)
{
  auto goal = frontier(3.0, 0.0);
  commitment_.commit(goal, 3.0, 0.0);
  auto unreachable = frontier(3.1, 0.0);
  unreachable->setAchievability(false);
  auto blacklisted = frontier(2.9, 0.0);
  blacklisted->setBlacklisted(true);

  const auto decision = keep({unreachable, blacklisted, frontier(-3.0, 0.0)}, 1.0);
  EXPECT_EQ(decision.outcome, Outcome::Lost);
  EXPECT_EQ(decision.frontier, goal);
  EXPECT_EQ(keep({frontier(3.0, 0.0)}, 2.0).outcome, Outcome::None);
}

TEST_F(GoalCommitmentTest, SlowProgressKeepsTheGoal)
{
  auto goal = frontier(5.0, 0.0);
  commitment_.commit(goal, 5.0, 0.0);
  // 0.25 m every 29 s is slow, but it is progress.
  for (int step = 1; step <= 6; ++step) {
    remaining_[goal] = 5.0 - 0.25 * step;
    EXPECT_EQ(keep({goal}, 29.0 * step).outcome, Outcome::Kept) << step;
  }
}

TEST_F(GoalCommitmentTest, GivesUpAfterTheStuckTimeoutWithoutProgress)
{
  auto goal = frontier(5.0, 0.0);
  commitment_.commit(goal, 5.0, 0.0);
  // Moving about without getting 0.25 m closer is not progress.
  remaining_[goal] = 4.8;
  EXPECT_EQ(keep({goal}, 10.0).outcome, Outcome::Kept);
  remaining_[goal] = 5.6;
  EXPECT_EQ(keep({goal}, 20.0).outcome, Outcome::Kept);
  remaining_[goal] = 4.9;
  EXPECT_EQ(keep({goal}, 29.9).outcome, Outcome::Kept);

  const auto decision = keep({goal}, 30.0);
  EXPECT_EQ(decision.outcome, Outcome::Stuck);
  EXPECT_EQ(decision.frontier, goal);
  EXPECT_EQ(keep({goal}, 31.0).outcome, Outcome::None);
}

TEST_F(GoalCommitmentTest, ProgressRestartsTheStuckTimer)
{
  auto goal = frontier(5.0, 0.0);
  commitment_.commit(goal, 5.0, 0.0);
  remaining_[goal] = 4.7;
  EXPECT_EQ(keep({goal}, 25.0).outcome, Outcome::Kept);
  EXPECT_EQ(keep({goal}, 54.9).outcome, Outcome::Kept);
  EXPECT_EQ(keep({goal}, 55.0).outcome, Outcome::Stuck);
}

TEST_F(GoalCommitmentTest, RefusesSettingsThatAreNotPositive)
{
  const auto remaining = [](const FrontierPtr &) {return 1.0;};
  for (const auto settings : {GoalCommitment::Settings{0.0, 30.0, 0.25},
      GoalCommitment::Settings{1.0, -1.0, 0.25},
      GoalCommitment::Settings{1.0, 30.0, std::numeric_limits<double>::quiet_NaN()}})
  {
    EXPECT_THROW(commitment_.keep({}, remaining, 0.0, settings), std::invalid_argument);
  }
}
}  // namespace
}  // namespace roadmap_explorer
