#ifndef GOAL_COMMITMENT_HPP_
#define GOAL_COMMITMENT_HPP_

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include <geometry_msgs/msg/point.hpp>

#include "roadmap_explorer/Frontier.hpp"

namespace roadmap_explorer
{

// Keeps one exploration goal across frontier searches. Each search re-detects
// the frontiers, and a frontier's goal point moves as the map grows, so the
// goal is identified by its position: the achievable frontier nearest the
// point first committed to, within match_radius of it.
//
// The goal is kept until no such frontier is left (it was reached, explored
// or became unreachable) or the robot has not got min_progress closer to it
// for stuck_timeout seconds. A goal given up that way is set aside for
// retry_cooldown seconds, or until no other frontier is left, and is given
// up for good on its max_attempts-th time.
class GoalCommitment
{
public:
  struct Settings
  {
    double match_radius;    // m
    double stuck_timeout;   // s
    double min_progress;    // m
    double retry_cooldown;  // s
    int64_t max_attempts;
  };

  enum class Outcome
  {
    None,       // nothing committed
    Kept,       // frontier continues the committed goal
    Lost,       // no achievable frontier is left at the committed goal
    SetAside,   // frontier is the committed goal, which made no progress in time
    GivenUp,    // as SetAside, on the last allowed attempt; frontiers within
                // match_radius of it are marked blacklisted
  };

  struct Decision
  {
    Outcome outcome;
    FrontierPtr frontier;
  };

  using Remaining = std::function<double(const FrontierPtr &)>;

  // Throws RoadmapExplorerException naming the first invalid setting.
  static void validate(const Settings & settings);

  // Marks the frontiers at goals set aside and still cooling down as
  // blacklisted for this search, unless that would leave none. Call it only
  // when choosing a new goal, after keep.
  void holdBack(const std::vector<FrontierPtr> & frontiers, double now, const Settings & settings);

  // remaining gives the travel distance from the robot to a frontier, and
  // now is a time in seconds. Any outcome but Kept ends the commitment.
  Decision keep(
    const std::vector<FrontierPtr> & frontiers, const Remaining & remaining, double now,
    const Settings & settings);

  void commit(const FrontierPtr & frontier, double remaining, double now);

  void clear();

private:
  struct SetAsideGoal
  {
    geometry_msgs::msg::Point point;
    int64_t attempts;
    double until;
  };

  std::optional<geometry_msgs::msg::Point> anchor_;
  FrontierPtr last_;
  double reference_ = 0.0;
  double progress_at_ = 0.0;
  std::vector<SetAsideGoal> set_aside_;
};

}  // namespace roadmap_explorer

#endif  // GOAL_COMMITMENT_HPP_
