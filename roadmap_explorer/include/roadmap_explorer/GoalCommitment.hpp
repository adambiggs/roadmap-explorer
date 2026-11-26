#ifndef GOAL_COMMITMENT_HPP_
#define GOAL_COMMITMENT_HPP_

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
// for stuck_timeout seconds.
class GoalCommitment
{
public:
  struct Settings
  {
    double match_radius;   // m
    double stuck_timeout;  // s
    double min_progress;   // m
  };

  enum class Outcome
  {
    None,   // nothing committed
    Kept,   // frontier continues the committed goal
    Lost,   // no achievable frontier is left at the committed goal
    Stuck,  // frontier is the committed goal, which made no progress in time
  };

  struct Decision
  {
    Outcome outcome;
    FrontierPtr frontier;
  };

  using Remaining = std::function<double(const FrontierPtr &)>;

  // remaining gives the travel distance from the robot to a frontier, and
  // now is a time in seconds. A Lost or Stuck decision ends the commitment.
  Decision keep(
    const std::vector<FrontierPtr> & frontiers, const Remaining & remaining, double now,
    const Settings & settings);

  void commit(const FrontierPtr & frontier, double remaining, double now);

  void clear();

private:
  std::optional<geometry_msgs::msg::Point> anchor_;
  FrontierPtr last_;
  double reference_ = 0.0;
  double progress_at_ = 0.0;
};

}  // namespace roadmap_explorer

#endif  // GOAL_COMMITMENT_HPP_
