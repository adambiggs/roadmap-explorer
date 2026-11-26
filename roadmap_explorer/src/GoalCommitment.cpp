#include "roadmap_explorer/GoalCommitment.hpp"

#include <cmath>
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
}  // namespace

GoalCommitment::Decision GoalCommitment::keep(
  const std::vector<FrontierPtr> & frontiers, const Remaining & remaining, double now,
  const Settings & settings)
{
  requirePositive(settings.match_radius, "goal_match_radius");
  requirePositive(settings.stuck_timeout, "goal_stuck_timeout");
  requirePositive(settings.min_progress, "goal_min_progress");
  if (!anchor_) {
    return {Outcome::None, nullptr};
  }

  FrontierPtr match;
  double nearest = std::numeric_limits<double>::infinity();
  for (const auto & frontier : frontiers) {
    if (!frontier->isAchievable() || frontier->isBlacklisted()) {
      continue;
    }
    const auto & point = frontier->getGoalPoint();
    const double offset = std::hypot(point.x - anchor_->x, point.y - anchor_->y);
    if (offset <= settings.match_radius && offset < nearest) {
      nearest = offset;
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
  } else if (now - progress_at_ >= settings.stuck_timeout) {
    clear();
    return {Outcome::Stuck, match};
  }
  return {Outcome::Kept, match};
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
