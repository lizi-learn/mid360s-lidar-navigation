#pragma once
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <vector>
namespace p2_nav_controller {
struct GaitPathBan {
  std::array<double, 2> anchor;
  double orientation;
  size_t index;
  double accumulated_angle;
};
inline std::optional<GaitPathBan>
gait_path_violation(const std::vector<std::array<double, 3>> &path,
                    const std::array<double, 3> &start,
                    const std::array<double, 3> &goal, double maximum,
                    double forward_reset, double terminal_region) {
  if (path.empty() ||
      !std::isfinite(maximum + forward_reset + terminal_region) ||
      maximum <= 0. || forward_reset <= 0. || terminal_region < 0.)
    throw std::invalid_argument("Invalid gait path contract");
  for (double value : start)
    if (!std::isfinite(value))
      throw std::invalid_argument("Invalid gait path start");
  for (double value : goal)
    if (!std::isfinite(value))
      throw std::invalid_argument("Invalid gait path goal");
  std::vector<bool> terminal(path.size());
  bool suffix = true;
  for (size_t i = path.size(); i-- > 0;) {
    for (double value : path[i])
      if (!std::isfinite(value))
        throw std::invalid_argument("Invalid gait path sample");
    suffix = suffix && std::hypot(path[i][0] - goal[0], path[i][1] - goal[1]) <=
                           terminal_region;
    terminal[i] = suffix;
  }
  auto previous = start;
  double angle = 0., translation = forward_reset;
  std::array<double, 2> anchor{start[0], start[1]};
  for (size_t i = 0; i < path.size(); ++i) {
    const auto &next = path[i];
    if (terminal[i] && std::hypot(previous[0] - goal[0],
                                  previous[1] - goal[1]) <= terminal_region)
      break;
    const double distance =
        std::hypot(next[0] - previous[0], next[1] - previous[1]);
    const double delta =
        std::remainder(next[2] - previous[2], 2. * std::acos(-1.));
    if (std::abs(delta) > 1e-5) {
      if (translation >= forward_reset - 1e-6) {
        angle = 0.;
        anchor = {previous[0], previous[1]};
      }
      translation = 0.;
      angle += std::abs(delta);
      if (angle > maximum + 1e-5)
        return GaitPathBan{{previous[0], previous[1]}, next[2], i, angle};
    } else
      translation += distance;
    previous = next;
  }
  return std::nullopt;
}
} // namespace p2_nav_controller
