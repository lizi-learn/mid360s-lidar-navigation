#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/node_lattice.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <iostream>
#include <link.h>
#include <string>
extern "C" void p2_lattice_clear_gait_exclusions();
extern "C" void p2_lattice_add_gait_exclusion(double, double, double, double);
extern "C" int p2_lattice_gait_edge_blocked(double, double, double, double);
int main(int argc, char **argv) {
  std::vector<std::string> loaded;
  dl_iterate_phdr(
      [](dl_phdr_info *info, size_t, void *opaque) {
        std::string name = info->dlpi_name;
        if (name.find("libnav2_smac_planner_lattice.so") != std::string::npos)
          static_cast<std::vector<std::string> *>(opaque)->push_back(name);
        return 0;
      },
      &loaded);
  if (argc != 3 || loaded.size() != 1 || loaded[0] != argv[2])
    throw std::runtime_error("Unexpected loaded gait SMAC library");
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>("gait_hook_probe");
  nav2_costmap_2d::Costmap2D map(240, 200, .05, 0., 0., 0);
  nav2_costmap_2d::Footprint fp;
  for (auto xy : std::vector<std::array<double, 2>>{
           {-.63, -.77}, {1.05, -.77}, {1.05, .77}, {-.63, .77}}) {
    geometry_msgs::msg::Point p;
    p.x = xy[0];
    p.y = xy[1];
    fp.push_back(p);
  }
  nav2_smac_planner::GridCollisionChecker check(&map, 72, node);
  check.setFootprint(fp, false, 1.);
  nav2_smac_planner::SearchInfo info{};
  if (argc != 3)
    throw std::runtime_error("Primitive path required");
  info.lattice_filepath = argv[1];
  info.allow_reverse_expansion = false;
  unsigned size = 240;
  auto &table = nav2_smac_planner::NodeLattice::motion_table;
  table.initMotionModel(size, info);
  nav2_smac_planner::NodeLattice from(0), to(1);
  from.setPose({78.f, 49.f, 7.f});
  to.setPose({78.f, 49.f, 8.f});
  nav2_smac_planner::MotionPrimitive *rotation = nullptr;
  for (auto p : table.getMotionPrimitives(&from))
    if (p->end_angle == 8 && p->trajectory_length < 1e-6)
      rotation = p;
  if (!rotation)
    throw std::runtime_error("No pure pivot");
  p2_lattice_clear_gait_exclusions();
  const bool original = to.isNodeValid(false, &check, rotation, false);
  p2_lattice_add_gait_exclusion(77.54, 48.66, 1.535, 10.);
  const int direct = p2_lattice_gait_edge_blocked(
      78., 49., table.getAngleFromBin(7), table.getAngleFromBin(8));
  const bool blocked = to.isNodeValid(false, &check, rotation, false);
  std::cout << "{\"original_valid\":" << original
            << ",\"direct_ban\":" << direct
            << ",\"node_valid_after_ban\":" << blocked << "}\n";
  if (!original || !direct || blocked)
    throw std::runtime_error("Gait exclusion hook failed");
  p2_lattice_clear_gait_exclusions();
  for (unsigned y = 0; y < 200; ++y)
    for (unsigned x = 0; x < 240; ++x)
      map.setCost(x, y, 253);
  map.setCost(100, 100, 130);
  if (check.inCollision(100.f, 100.f, 0.f, false) || check.getCost() != 130.f)
    throw std::runtime_error(
        "Valid full footprint lost center traversal gradient");
  map.setCost(121, 100, 254);
  if (!check.inCollision(100.f, 100.f, 0.f, false))
    throw std::runtime_error(
        "Center cost must not override full footprint collision");
  node.reset();
  rclcpp::shutdown();
}
