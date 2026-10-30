#include <stdexcept>
template <class T> void require(T &&value) {
  if (!static_cast<bool>(value))
    throw std::runtime_error("Gait path contract failed");
}
#include "p2_nav_controller/gait_path.hpp"
#include <cmath>
#include <iostream>
#include <thread>
extern "C" void p2_lattice_clear_gait_exclusions();
extern "C" void p2_lattice_add_gait_exclusion(double, double, double, double);
extern "C" int p2_lattice_gait_edge_blocked(double, double, double, double);
using Pose = std::array<double, 3>;
void rotate(std::vector<Pose> &path, double angle) {
  const auto from = path.back();
  int n = std::ceil(std::abs(angle) / .02);
  for (int i = 1; i <= n; ++i)
    path.push_back({from[0], from[1], from[2] + angle * i / n});
}
void forward(std::vector<Pose> &path, double distance) {
  const auto from = path.back();
  int n = std::ceil(distance / .0125);
  for (int i = 1; i <= n; ++i)
    path.push_back({from[0] + distance * i / n * std::cos(from[2]),
                    from[1] + distance * i / n * std::sin(from[2]), from[2]});
}
int main() {
  const double pi = std::acos(-1.);
  const Pose start{2, 2, 0}, goal{10, 10, 0};
  auto violation = [&](const std::vector<Pose> &p) {
    return p2_nav_controller::gait_path_violation(p, start, goal, pi / 2, .5,
                                                  .3);
  };
  std::vector<Pose> p{start};
  rotate(p, 3 * pi / 4);
  require(violation(p));
  p = {start};
  rotate(p, pi / 2);
  forward(p, .5);
  rotate(p, pi / 4);
  require(!violation(p));
  p = {start};
  rotate(p, pi / 2);
  forward(p, .49);
  rotate(p, pi / 4);
  require(violation(p));
  p = {start};
  rotate(p, pi / 4);
  rotate(p, -pi / 4);
  rotate(p, pi / 4);
  require(violation(p));
  p = {start};
  rotate(p, 3 * pi / 4);
  require(
      !p2_nav_controller::gait_path_violation(p, start, start, pi / 2, .5, .3));
  p = {start};
  rotate(p, pi / 4);
  forward(p, .4);
  rotate(p, pi / 4);
  forward(p, .4);
  rotate(p, pi / 4);
  auto local_ban = violation(p);
  require(local_ban && std::hypot(local_ban->anchor[0] - start[0],
                                  local_ban->anchor[1] - start[1]) > .5);
  const auto edge = p[local_ban->index - 1];
  require(std::hypot(local_ban->anchor[0] - edge[0],
                     local_ban->anchor[1] - edge[1]) < 1e-12);
  bool rejected = false;
  try {
    violation({});
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  require(rejected);
  p2_lattice_clear_gait_exclusions();
  p2_lattice_add_gait_exclusion(1., 2., 100 * pi / 180., 5.);
  require(!p2_lattice_gait_edge_blocked(1., 2., pi / 2, 97 * pi / 180.));
  require(
      p2_lattice_gait_edge_blocked(1., 2., 97 * pi / 180., 116 * pi / 180.));
  require(
      !p2_lattice_gait_edge_blocked(7., 2., 97 * pi / 180., 116 * pi / 180.));
  std::thread other([&] {
    require(
        !p2_lattice_gait_edge_blocked(1., 2., 97 * pi / 180., 116 * pi / 180.));
  });
  other.join();
  p2_lattice_clear_gait_exclusions();
  require(
      !p2_lattice_gait_edge_blocked(1., 2., 97 * pi / 180., 116 * pi / 180.));
  std::cout << "Gait rotation budget, real forward leg, terminal scope, empty "
               "input and query/thread isolation PASS\n";
}
