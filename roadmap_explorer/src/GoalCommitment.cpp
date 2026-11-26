#include "roadmap_explorer/GoalCommitment.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>

namespace roadmap_explorer
{
namespace
{
void requirePositive(double value, const char * name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string("fullPathOptimizer.") + name +
      " must be finite and positive");
  }
}

double offset(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

bool available(const FrontierPtr & frontier)
{
  return frontier->isAchievable() && !frontier->isBlacklisted();
}
}  // namespace

void GoalCommitment::validate(const Settings & settings)
{
  requirePositive(settings.match_radius, "goal_match_radius");
  requirePositive(settings.stuck_timeout, "goal_stuck_timeout");
  requirePositive(settings.min_progress, "goal_min_progress");
  requirePositive(settings.retry_cooldown, "goal_retry_cooldown");
  if (settings.max_attempts < 1) {
    throw std::invalid_argument("fullPathOptimizer.goal_max_attempts must be at least 1");
  }
}

void GoalCommitment::holdBack(
  const std::vector<FrontierPtr> & frontiers, double now, const Settings & settings)
{
  validate(settings);
  std::vector<FrontierPtr> held;
  bool other_left = false;
  for (const auto & frontier : frontiers) {
    if (!available(frontier)) {
      continue;
    }
    const auto cooling = std::any_of(
      set_aside_.begin(), set_aside_.end(), [&](const SetAsideGoal & goal) {
        return now < goal.until &&
               offset(frontier->getGoalPoint(), goal.point) <= settings.match_radius;
      });
    if (cooling) {
      held.push_back(frontier);
    } else {
      other_left = true;
    }
  }
  // With nothing else left to explore, retry a goal set aside early rather
  // than finish exploration without it.
  if (other_left) {
    for (auto & frontier : held) {
      frontier->setBlacklisted(true);
    }
  }
}

GoalCommitment::Decision GoalCommitment::keep(
  const std::vector<FrontierPtr> & frontiers, const Remaining & remaining, double now,
  const Settings & settings)
{
  validate(settings);
  if (!anchor_) {
    return {Outcome::None, nullptr};
  }

  FrontierPtr match;
  double nearest = std::numeric_limits<double>::infinity();
  for (const auto & frontier : frontiers) {
    if (!available(frontier)) {
      continue;
    }
    const double d = offset(frontier->getGoalPoint(), *anchor_);
    if (d <= settings.match_radius && d < nearest) {
      nearest = d;
      match = frontier;
    }
  }
  if (!match) {
    auto lost = last_;
    clear();
    return {Outcome::Lost, lost};
  }

  last_ = match;
  const double distance = remaining(match);
  if (distance <= reference_ - settings.min_progress) {
    reference_ = distance;
    progress_at_ = now;
    return {Outcome::Kept, match};
  }
  if (now - progress_at_ < settings.stuck_timeout) {
    return {Outcome::Kept, match};
  }

  clear();
  const auto & point = match->getGoalPoint();
  auto earlier = std::find_if(
    set_aside_.begin(), set_aside_.end(), [&](const SetAsideGoal & goal) {
      return offset(point, goal.point) <= settings.match_radius;
    });
  if (earlier == set_aside_.end()) {
    set_aside_.push_back({point, 0, 0.0});
    earlier = std::prev(set_aside_.end());
  }
  earlier->point = point;
  earlier->attempts += 1;
  earlier->until = now + settings.retry_cooldown;
  if (earlier->attempts >= settings.max_attempts) {
    set_aside_.erase(earlier);
    return {Outcome::GivenUp, match};
  }
  return {Outcome::SetAside, match};
}

void GoalCommitment::commit(const FrontierPtr & frontier, double remaining, double now)
{
  anchor_ = frontier->getGoalPoint();
  last_ = frontier;
  reference_ = remaining;
  progress_at_ = now;
}

void GoalCommitment::clear()
{
  anchor_.reset();
  last_.reset();
}

}  // namespace roadmap_explorer
