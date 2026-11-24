#pragma once

#include <functional>
#include <optional>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace roadmap_explorer
{
// A current costmap window shared by all disconnected candidates in one search.
class OneTurnSearch : private nav2_costmap_2d::Costmap2D
{
public:
  using Point = geometry_msgs::msg::Point;
  OneTurnSearch(nav2_costmap_2d::Costmap2D & source, const Point & robot,
    double radius, const std::function<bool()> & canceled);
  std::optional<double> connectionLength(const Point & start, const Point & end,
    const Point & goal);

private:
  int cell(const Point & p) const;
  std::optional<unsigned int> unknownOnRay(int from, int to);
  struct Turn {
    Point point;
    int cell;
    double robot_distance;
    double start_distance;
    unsigned int robot_unknown;
    unsigned int start_unknown;
  };
  void collectTurns(const Point & start);


  double source_x_ = 0.0, source_y_ = 0.0;
  unsigned int offset_x_ = 0, offset_y_ = 0;
  const Point robot_;
  const double radius_;
  const std::function<bool()> canceled_;
  std::vector<unsigned int> minimum_unknown_;
  std::vector<unsigned int> minimum_steps_;
  std::vector<Turn> turns_;
  std::optional<Point> cached_start_;
};
}
