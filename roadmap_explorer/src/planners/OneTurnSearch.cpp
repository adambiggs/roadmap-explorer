#include "roadmap_explorer/planners/OneTurnSearch.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>

#include <nav2_util/line_iterator.hpp>

namespace roadmap_explorer
{
namespace
{
double distance(const OneTurnSearch::Point & a, const OneTurnSearch::Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}
}

OneTurnSearch::OneTurnSearch(nav2_costmap_2d::Costmap2D & source, const Point & robot,
  double radius, const std::function<bool()> & canceled)
: robot_(robot), radius_(radius), canceled_(canceled)
{
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*source.getMutex());
    unsigned int x, y;
    if (!source.worldToMap(robot.x, robot.y, x, y) || canceled_()) {
      return;
    }
    int xmin, ymin, xmax, ymax;
    source.worldToMapEnforceBounds(robot.x - radius, robot.y - radius, xmin, ymin);
    source.worldToMapEnforceBounds(robot.x + radius, robot.y + radius, xmax, ymax);
    xmax = std::min(xmax, static_cast<int>(source.getSizeInCellsX()) - 1);
    ymax = std::min(ymax, static_cast<int>(source.getSizeInCellsY()) - 1);
    source_x_ = source.getOriginX();
    source_y_ = source.getOriginY();
    offset_x_ = xmin;
    offset_y_ = ymin;
    const double res = source.getResolution();
    resizeMap(xmax - xmin + 1, ymax - ymin + 1, res,
      source.getOriginX() + xmin * res, source.getOriginY() + ymin * res);
    for (int row = ymin; row <= ymax; ++row) {
      std::copy_n(source.getCharMap() + source.getIndex(xmin, row), getSizeInCellsX(),
        getCharMap() + getIndex(0, row - ymin));
    }
  }

  const int seed = cell(robot);
  if (seed < 0 || getCost(seed) == 253 || getCost(seed) == 254) {
    return;
  }
  // Any Bresenham segment is an 8-connected walk. Its cheapest possible
  // unknown-cell count rules out barriers before testing any candidate rays.
  const auto width = getSizeInCellsX();
  const auto height = getSizeInCellsY();
  const double allowance = radius_ / getResolution() * 0.3;
  const auto lowerBounds = [&](bool count_unknown) {
    std::vector<unsigned int> minimum(width * height, std::numeric_limits<unsigned int>::max());
    minimum[seed] = count_unknown && getCost(seed) == 255;
    std::deque<int> pending{seed};
    const double limit = count_unknown ? allowance : radius_ / getResolution() + 1;
    while (!pending.empty() && !canceled_()) {
      const int from = pending.front();
      pending.pop_front();
      const int cx = from % width, cy = from / width;
      for (int y = std::max(0, cy - 1); y <= std::min(static_cast<int>(height) - 1, cy + 1); ++y) {
        for (int x = std::max(0, cx - 1); x <= std::min(static_cast<int>(width) - 1, cx + 1); ++x) {
          const int to = getIndex(x, y);
          const auto cost = getCost(to);
          const bool charge = !count_unknown || cost == 255;
          const auto bound = minimum[from] + charge;
          if (cost == 253 || cost == 254 || bound >= minimum[to] || bound > limit) {
            continue;
          }
          minimum[to] = bound;
          if (charge) {
            pending.push_back(to);
          } else {
            pending.push_front(to);
          }
        }
      }
    }
    return minimum;
  };
  minimum_unknown_ = lowerBounds(true);
  minimum_steps_ = lowerBounds(false);
}

int OneTurnSearch::cell(const Point & p) const
{
  if (!getSizeInCellsX()) {
    return -1;
  }
  // Convert in the source frame before subtracting the integer crop offset.
  // A translated floating origin can round the same point into an adjacent cell.
  const double x = (p.x - source_x_) / getResolution();
  const double y = (p.y - source_y_) / getResolution();
  if (!(x >= offset_x_ && x < offset_x_ + getSizeInCellsX() &&
    y >= offset_y_ && y < offset_y_ + getSizeInCellsY())) {
    return -1;
  }
  return static_cast<int>(getIndex(static_cast<unsigned int>(x) - offset_x_,
    static_cast<unsigned int>(y) - offset_y_));
}

std::optional<unsigned int> OneTurnSearch::unknownOnRay(int from, int to)
{
  unsigned int x0, y0, x1, y1;
  indexToCells(from, x0, y0);
  indexToCells(to, x1, y1);
  const auto * costs = getCharMap();
  const unsigned int width = getSizeInCellsX();
  const auto trace = [&](unsigned int ax, unsigned int ay, unsigned int bx, unsigned int by)
    -> std::optional<unsigned int> {
    unsigned int unknown = 0;
    for (nav2_util::LineIterator ray(ax, ay, bx, by); ray.isValid(); ray.advance()) {
      const auto cost = costs[ray.getY() * width + ray.getX()];
      if (cost == 253 || cost == 254) {
        return std::nullopt;
      }
      unknown += cost == 255;
    }
    return unknown;
  };
  const auto a = trace(x0, y0, x1, y1);
  if (!a) {return std::nullopt;}
  const auto b = trace(x1, y1, x0, y0);
  return b ? std::optional<unsigned int>{std::max(*a, *b)} : std::nullopt;
}

void OneTurnSearch::collectTurns(const Point & start)
{
  cached_start_ = start;
  turns_.clear();
  const int robot_cell = cell(robot_), start_cell = cell(start);
  if (start_cell < 0) {
    return;
  }
  const double allowance = radius_ / getResolution() * 0.3;
  for (size_t i = 0; i < minimum_unknown_.size(); ++i) {
    if (i % getSizeInCellsX() == 0 && canceled_()) {return;}
    if (minimum_unknown_[i] > allowance || getCost(i) >= 253 ||
      int(i) == robot_cell || int(i) == start_cell) {
      continue;
    }
    unsigned int x, y;
    indexToCells(i, x, y);
    Point turn;
    turn.x = source_x_ + (offset_x_ + x + 0.5) * getResolution();
    turn.y = source_y_ + (offset_y_ + y + 0.5) * getResolution();
    const double dr = distance(robot_, turn), ds = distance(start, turn);
    if (std::max(dr, ds) >= radius_) {
      continue;
    }
    const auto ru = unknownOnRay(robot_cell, i);
    if (!ru || *ru > allowance) {
      continue;
    }
    const auto su = robot_cell == start_cell ? ru : unknownOnRay(start_cell, i);
    if (su && *su <= allowance) {
      turns_.push_back({turn, int(i), dr, ds, *ru, *su});
    }
  }
}

std::optional<double> OneTurnSearch::connectionLength(
  const Point & start, const Point & end, const Point & goal)
{
  const int goal_cell = cell(goal), end_cell = cell(end);
  if (minimum_unknown_.empty() || goal_cell < 0 || end_cell < 0 || canceled_() ||
    distance(robot_, goal) >= radius_ || distance(start, end) >= radius_) {
    return std::nullopt;
  }
  const double allowance = radius_ / getResolution() * 0.3;
  // Each ray takes max(|dx|, |dy|) grid steps. Its world length is at least
  // (steps - 0.5) * resolution because the bend is at a cell center. Two rays
  // can therefore save at most one cell between their off-center endpoints.
  if (minimum_unknown_[goal_cell] > allowance ||
    (double(minimum_steps_[goal_cell]) - 1.0) * getResolution() >= radius_) {
    return std::nullopt;
  }
  if (!cached_start_ || *cached_start_ != start) {
    collectTurns(start);
  }
  double best = radius_;
  int next_row = -1;
  for (const auto & turn : turns_) {
    if (turn.cell >= next_row) {
      if (canceled_()) {return std::nullopt;}
      next_row = turn.cell - turn.cell % getSizeInCellsX() + getSizeInCellsX();
    }
    const double length = turn.robot_distance + distance(turn.point, goal);
    if (length >= best || turn.start_distance + distance(turn.point, end) >= radius_ ||
      turn.cell == goal_cell || turn.cell == end_cell) {
      continue;
    }
    const auto gu = unknownOnRay(turn.cell, goal_cell);
    if (!gu || turn.robot_unknown + *gu > allowance) {
      continue;
    }
    const auto eu = end_cell == goal_cell ? gu : unknownOnRay(turn.cell, end_cell);
    if (eu && turn.start_unknown + *eu <= allowance) {
      best = length;
    }
  }
  return best < radius_ ? std::optional<double>{best} : std::nullopt;
}
}
