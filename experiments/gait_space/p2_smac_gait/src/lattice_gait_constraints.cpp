#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
namespace {
struct GaitExclusion {
  double x, y, yaw, radius;
};
thread_local std::vector<GaitExclusion> exclusions;
thread_local unsigned calls = 0, blocked = 0, nearby = 0;
} // namespace
extern "C" {
void p2_lattice_clear_gait_exclusions() {
  exclusions.clear();
  calls = blocked = nearby = 0;
}
void p2_lattice_gait_stats(unsigned *out) {
  out[0] = calls;
  out[1] = nearby;
  out[2] = blocked;
  out[3] = exclusions.size();
}
void p2_lattice_add_gait_exclusion(double x, double y, double yaw,
                                   double radius) {
  if (!std::isfinite(x + y + yaw + radius) || radius <= 0. ||
      exclusions.size() >= 32)
    throw std::invalid_argument("Invalid bounded gait search exclusion");
  exclusions.push_back({x, y, yaw, radius});
}
int p2_lattice_gait_edge_blocked(double x, double y, double from, double to) {
  ++calls;
  if (!std::isfinite(x + y + from + to))
    return 1;
  const double delta = std::remainder(to - from, 2. * std::acos(-1.));
  for (const auto &ban : exclusions) {
    if (std::hypot(x - ban.x, y - ban.y) >= ban.radius)
      continue;
    ++nearby;
    const double relative = std::remainder(ban.yaw - from, 2. * std::acos(-1.));
    if (delta >= 0. ? relative >= -1e-6 && relative <= delta + 1e-6
                    : relative <= 1e-6 && relative >= delta - 1e-6) {
      ++blocked;
      return 1;
    }
  }
  return 0;
}
}
