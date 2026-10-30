#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_smac_planner/node_lattice.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <link.h>
#include <stdexcept>
void require(bool condition) {
  if (!condition)
    throw std::runtime_error("Continuous lattice collision regression failed");
}
int main(int argc, char **argv) {
  require(argc == 3);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "lattice_sweep_regression");
  nav2_costmap_2d::Costmap2D map(120, 120, .05, 0., 0., 0);
  map.setCost(50, 50, 2);
  for (int x = 62; x <= 64; ++x)
    for (int y = 71; y <= 73; ++y)
      map.setCost(x, y, 254);
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
  const bool start = check.inCollision(50.f, 50.f, 0.f, false),
             end = check.inCollision(50.f, 50.f, 9.f, false);
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
      independent(&map);
  const double middle =
      independent.footprintCostAtPose(2.525, 2.525, std::acos(-1.) / 8., fp);
  require(!start && !end);
  require(middle >= 254.);
  nav2_smac_planner::SearchInfo info{};
  info.lattice_filepath = argv[2];
  info.allow_reverse_expansion = false;
  info.rotation_penalty = 5.;
  info.cost_penalty = 2.;
  info.non_straight_penalty = 1.2;
  unsigned size = 120;
  nav2_smac_planner::NodeLattice::motion_table.initMotionModel(size, info);
  nav2_smac_planner::NodeLattice from(0), to(1);
  from.setPose({50.f, 50.f, 0.f});
  to.setPose({50.f, 50.f, 1.f});
  nav2_smac_planner::MotionPrimitive *rotation = nullptr;
  for (auto p :
       nav2_smac_planner::NodeLattice::motion_table.getMotionPrimitives(&from))
    if (p->end_angle == 1 && p->trajectory_length < 1e-6)
      rotation = p;
  require(rotation);
  const bool accepted = to.isNodeValid(false, &check, rotation, false);
  std::cout << "{\"endpoints_clear\":true,\"intermediate_independent_cost\":"
            << middle
            << ",\"transition_accepted\":" << (accepted ? "true" : "false")
            << "}\n";
  std::vector<std::string> libraries;
  dl_iterate_phdr(
      [](dl_phdr_info *info, size_t, void *data) {
        std::string name = info->dlpi_name;
        if (name.find("libnav2_smac_planner_") != std::string::npos)
          static_cast<std::vector<std::string> *>(data)->push_back(name);
        return 0;
      },
      &libraries);
  require(libraries.size() == 1);
  const bool patched = std::string(argv[1]) == "patched";
  require(patched
              ? libraries[0].find("p2_smac_continuous/lib/") !=
                    std::string::npos
              : libraries[0].find("/opt/ros/humble/lib/") != std::string::npos);
  std::cout << "Loaded lattice: " << libraries[0] << "\n";
  const bool expect_accepted = argc > 1 && std::string(argv[1]) == "stock";
  require(accepted == expect_accepted);
  node.reset();
  rclcpp::shutdown();
}
