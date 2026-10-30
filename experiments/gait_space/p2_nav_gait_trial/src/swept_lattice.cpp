#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_smac_planner/smac_planner_lattice.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "p2_nav_controller/gait_path.hpp"
#include "p2_nav_controller/terminal_path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
extern "C" void p2_lattice_clear_gait_exclusions();
extern "C" void p2_lattice_gait_stats(unsigned *);
extern "C" void p2_lattice_add_gait_exclusion(double, double, double, double);

namespace p2_nav_controller {
// Preserve the requested final pose after the lattice's quantization.
// P2 can pivot; every interpolated terminal orientation must fit the planning
// clearance footprint. The short terminal connection is also swept checked by
// the outer planner against the original map before it can be returned.
class GaitTerminalLattice : public nav2_smac_planner::SmacPlannerLattice {
public:
  nav_msgs::msg::Path
  createPlan(const geometry_msgs::msg::PoseStamped &start,
             const geometry_msgs::msg::PoseStamped &goal) override {
    if (start.header.frame_id == goal.header.frame_id) {
      const auto &a = start.pose.orientation;
      const auto &b = goal.pose.orientation;
      const double an =
          std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w);
      const double bn =
          std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z + b.w * b.w);
      if (!std::isfinite(an) || !std::isfinite(bn) || an < .8 || an > 1.2 ||
          bn < .8 || bn > 1.2)
        throw nav2_core::PlannerException("Invalid short-path quaternion");
      auto samples = terminal_path(
          {start.pose.position.x, start.pose.position.y, tf2::getYaw(a)},
          {goal.pose.position.x, goal.pose.position.y, tf2::getYaw(b)});
      if (!samples.empty()) {
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
            *(_costmap->getMutex()));
        nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
            checker(_costmap);
        bool clear = true;
        for (const auto &p : samples) {
          const double cost = checker.footprintCostAtPose(
              p[0], p[1], p[2], _costmap_ros->getRobotFootprint());
          if (!std::isfinite(cost) ||
              cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
            clear = false;
            break;
          }
        }
        if (clear) {
          nav_msgs::msg::Path short_path;
          short_path.header = goal.header;
          for (const auto &p : samples) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = short_path.header;
            pose.pose.position.x = p[0];
            pose.pose.position.y = p[1];
            pose.pose.orientation =
                nav2_util::geometry_utils::orientationAroundZAxis(p[2]);
            short_path.poses.push_back(pose);
          }
          return short_path;
        }
      }
    }
    auto path = SmacPlannerLattice::createPlan(start, goal);
    if (path.poses.empty())
      return path;
    if (goal.header.frame_id != path.header.frame_id)
      throw nav2_core::PlannerException(
          "Exact terminal heading requires the map frame");
    const auto &q = goal.pose.orientation;
    const double norm =
        std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!std::isfinite(norm) || norm < .8 || norm > 1.2)
      throw nav2_core::PlannerException(
          "Invalid requested terminal quaternion");
    const auto end = path.poses.back();
    const auto samples = terminal_path(
        {end.pose.position.x, end.pose.position.y,
         tf2::getYaw(end.pose.orientation)},
        {goal.pose.position.x, goal.pose.position.y, tf2::getYaw(q)});
    if (samples.empty())
      throw nav2_core::PlannerException(
          "Lattice endpoint too far from requested goal");
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
        *(_costmap->getMutex()));
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
        checker(_costmap);
    for (const auto &p : samples) {
      const double cost = checker.footprintCostAtPose(
          p[0], p[1], p[2], _costmap_ros->getRobotFootprint());
      if (!std::isfinite(cost) || cost >= nav2_costmap_2d::LETHAL_OBSTACLE)
        throw nav2_core::PlannerException(
            "Requested terminal connection lacks planning clearance");
    }
    for (size_t i = 1; i < samples.size(); ++i) {
      auto pose = end;
      pose.pose.position.x = samples[i][0];
      pose.pose.position.y = samples[i][1];
      pose.pose.orientation =
          nav2_util::geometry_utils::orientationAroundZAxis(samples[i][2]);
      path.poses.push_back(pose);
    }
    path.poses.back().pose = goal.pose;
    return path;
  }
};
} // namespace p2_nav_controller
namespace p2_nav_controller {
// Search exclusions constrain reference centers only. INSCRIBED (253) rejects a
// lattice center without inventing a physical lethal obstacle along the
// footprint. All returned states are checked against the original unmodified
// map.
class GaitFeasibleLattice : public GaitTerminalLattice {
  double max_rotation_ = std::acos(-1.) / 2., forward_reset_ = .5,
         terminal_region_ = .3;
  std::mutex planning_mutex_;
  using P = std::array<double, 3>;
  std::vector<std::array<double, 2>> interior_;
  nav2_costmap_2d::Footprint corners_;
  std::vector<int> prefix_;
  unsigned stride_ = 0;
  bool logged_resolution_ = false;
  bool clear(const P &p, nav2_costmap_2d::Costmap2D &map) {
    if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]))
      return false;
    const double c = std::cos(p[2]), s = std::sin(p[2]);
    double lx = 1e9, ly = 1e9, hx = -1e9, hy = -1e9;
    for (const auto &v : corners_) {
      double x = p[0] + c * v.x - s * v.y, y = p[1] + s * v.x + c * v.y;
      lx = std::min(lx, x);
      ly = std::min(ly, y);
      hx = std::max(hx, x);
      hy = std::max(hy, y);
    }
    unsigned x0, y0, x1, y1;
    if (!map.worldToMap(lx, ly, x0, y0) || !map.worldToMap(hx, hy, x1, y1))
      return false;
    const int occupied =
        prefix_[(y1 + 1) * stride_ + x1 + 1] - prefix_[y0 * stride_ + x1 + 1] -
        prefix_[(y1 + 1) * stride_ + x0] + prefix_[y0 * stride_ + x0];
    if (occupied == 0)
      return true; // Entire enclosing rectangle contains no lethal/unknown
                   // cell.
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
        check(&map);
    const double boundary = check.footprintCostAtPose(
        p[0], p[1], p[2], _costmap_ros->getRobotFootprint());
    if (!std::isfinite(boundary) || boundary < 0 || boundary >= 254)
      return false;
    for (const auto &v : interior_) {
      unsigned x, y;
      if (!map.worldToMap(p[0] + c * v[0] - s * v[1],
                          p[1] + s * v[0] + c * v[1], x, y) ||
          map.getCost(x, y) >= 254)
        return false;
    }
    return true;
  }
  void footprint_samples() {
    const auto fp = _costmap_ros->getRobotFootprint();
    if (fp.size() != 4)
      throw nav2_core::PlannerException(
          "Swept trial requires its explicit rectangular footprint");
    double lx = 1e9, ly = 1e9, hx = -1e9, hy = -1e9;
    for (const auto &p : fp) {
      lx = std::min(lx, p.x);
      ly = std::min(ly, p.y);
      hx = std::max(hx, p.x);
      hy = std::max(hy, p.y);
    }
    for (const auto &p : fp)
      if ((std::abs(p.x - lx) > 1e-8 && std::abs(p.x - hx) > 1e-8) ||
          (std::abs(p.y - ly) > 1e-8 && std::abs(p.y - hy) > 1e-8))
        throw nav2_core::PlannerException("Nonrectangular trial footprint");
    corners_ = fp;
    interior_.clear();
    const int nx = std::ceil((hx - lx) / .025),
              ny = std::ceil((hy - ly) / .025);
    for (int i = 0; i <= nx; ++i)
      for (int j = 0; j <= ny; ++j)
        interior_.push_back({lx + (hx - lx) * i / nx, ly + (hy - ly) * j / ny});
  }
  std::vector<P> violations(const nav_msgs::msg::Path &path,
                            const geometry_msgs::msg::PoseStamped &start,
                            nav2_costmap_2d::Costmap2D &map) {
    std::vector<P> bad;
    P from{start.pose.position.x, start.pose.position.y,
           tf2::getYaw(start.pose.orientation)};
    if (!clear(from, map))
      bad.push_back(from);
    for (const auto &pose : path.poses) {
      P to{pose.pose.position.x, pose.pose.position.y,
           tf2::getYaw(pose.pose.orientation)};
      const double da = std::remainder(to[2] - from[2], 2. * std::acos(-1.));
      const int n =
          std::max({1,
                    static_cast<int>(std::ceil(
                        std::hypot(to[0] - from[0], to[1] - from[1]) / .025)),
                    static_cast<int>(std::ceil(std::abs(da) / .02))});
      for (int j = 1; j <= n; ++j) {
        P p{from[0] + (to[0] - from[0]) * j / n,
            from[1] + (to[1] - from[1]) * j / n, from[2] + da * j / n};
        if (!clear(p, map))
          bad.push_back(p);
      }
      from = to;
    }
    return bad;
  }
  bool add_gait_ban(const nav_msgs::msg::Path &path,
                    const geometry_msgs::msg::PoseStamped &start,
                    const geometry_msgs::msg::PoseStamped &goal,
                    nav2_costmap_2d::Costmap2D &map) {
    std::vector<P> samples;
    for (const auto &pose : path.poses)
      samples.push_back({pose.pose.position.x, pose.pose.position.y,
                         tf2::getYaw(pose.pose.orientation)});
    const auto ban =
        gait_path_violation(samples,
                            {start.pose.position.x, start.pose.position.y,
                             tf2::getYaw(start.pose.orientation)},
                            {goal.pose.position.x, goal.pose.position.y,
                             tf2::getYaw(goal.pose.orientation)},
                            max_rotation_, forward_reset_, terminal_region_);
    if (!ban)
      return false;
    p2_lattice_add_gait_exclusion(
        (ban->anchor[0] - map.getOriginX()) / map.getResolution() - .5,
        (ban->anchor[1] - map.getOriginY()) / map.getResolution() - .5,
        ban->orientation, (forward_reset_ - 1e-5) / map.getResolution());
    RCLCPP_WARN(_logger,
                "Gait turn exclusion at %.3f %.3f yaw %.3f accumulated %.3f; "
                "require %.3fm forward leg",
                ban->anchor[0], ban->anchor[1], ban->orientation,
                ban->accumulated_angle, forward_reset_);
    return true;
  }

public:
  void activate() override {
    GaitTerminalLattice::activate();
    _dyn_params_handler.reset();
    auto node = _node.lock();
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    for (const auto &item : std::vector<std::pair<std::string, double>>{
             {"gait_max_midroute_rotation", max_rotation_},
             {"gait_min_forward_leg", forward_reset_},
             {"gait_terminal_region", terminal_region_}})
      if (!node->has_parameter(_name + "." + item.first))
        node->declare_parameter(_name + "." + item.first,
                                rclcpp::ParameterValue(item.second), read_only);
    node->get_parameter(_name + ".gait_max_midroute_rotation", max_rotation_);
    node->get_parameter(_name + ".gait_min_forward_leg", forward_reset_);
    node->get_parameter(_name + ".gait_terminal_region", terminal_region_);
    if (!std::isfinite(max_rotation_ + forward_reset_ + terminal_region_) ||
        max_rotation_ <= 0. || max_rotation_ > std::acos(-1.) / 2. + 1e-8 ||
        forward_reset_ < .5 || terminal_region_ < 0. || terminal_region_ > .3)
      throw nav2_core::PlannerException(
          "Gait bounds exceed reviewed model contract");
    _dyn_params_handler = node->add_on_set_parameters_callback(
        [this](std::vector<rclcpp::Parameter> p) {
          std::lock_guard<std::mutex> lock(planning_mutex_);
          return dynamicParametersCallback(p);
        });
  }
  nav_msgs::msg::Path
  createPlan(const geometry_msgs::msg::PoseStamped &start,
             const geometry_msgs::msg::PoseStamped &goal) override {
    std::lock_guard<std::mutex> serial(planning_mutex_);
    p2_lattice_clear_gait_exclusions();
    struct ClearGaitQuery {
      ~ClearGaitQuery() { p2_lattice_clear_gait_exclusions(); }
    } clear_gait_query;
    auto *original = _costmap;
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> snapshot(
        *original->getMutex());
    nav2_costmap_2d::Costmap2D search(*original);
    footprint_samples();
    stride_ = original->getSizeInCellsX() + 1;
    prefix_.assign(stride_ * (original->getSizeInCellsY() + 1), 0);
    for (unsigned y = 0; y < original->getSizeInCellsY(); ++y)
      for (unsigned x = 0; x < original->getSizeInCellsX(); ++x)
        prefix_[(y + 1) * stride_ + x + 1] =
            (original->getCost(x, y) >= 254) + prefix_[y * stride_ + x + 1] +
            prefix_[(y + 1) * stride_ + x] - prefix_[y * stride_ + x];
    if (!logged_resolution_) {
      RCLCPP_INFO(_logger, "Actual checked costmap resolution %.17g",
                  original->getResolution());
      logged_resolution_ = true;
    }
    const auto saved_checker = _collision_checker;
    struct Restore {
      GaitFeasibleLattice *self;
      nav2_costmap_2d::Costmap2D *map;
      nav2_smac_planner::GridCollisionChecker checker;
      ~Restore() {
        self->_costmap = map;
        self->_collision_checker = checker;
        self->_a_star->setCollisionChecker(&self->_collision_checker);
      }
    } restore{this, original, saved_checker};
    _costmap = &search;
    _collision_checker =
        nav2_smac_planner::GridCollisionChecker(&search, 72u, _node.lock());
    const auto began = std::chrono::steady_clock::now();
    for (int attempt = 0; attempt < 8; ++attempt) {
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        began)
              .count() > 6.)
        break;
      auto path = GaitTerminalLattice::createPlan(start, goal);
      unsigned stats[4];
      p2_lattice_gait_stats(stats);
      RCLCPP_INFO(_logger,
                  "Gait hook stats calls=%u nearby=%u blocked=%u exclusions=%u",
                  stats[0], stats[1], stats[2], stats[3]);
      if (path.poses.empty())
        break;
      auto bad = violations(path, start, *original);
      if (bad.empty()) {
        if (add_gait_ban(path, start, goal, *original))
          continue;
        RCLCPP_INFO(_logger, "Swept path accepted after %d repair searches",
                    attempt);
        return path;
      }
      RCLCPP_WARN(_logger,
                  "Swept path rejected: %zu envelope samples, repair %d",
                  bad.size(), attempt);
      auto smoother = std::move(_smoother);
      try {
        path = GaitTerminalLattice::createPlan(start, goal);
      } catch (...) {
        _smoother = std::move(smoother);
        throw;
      }
      _smoother = std::move(smoother);
      if (path.poses.empty())
        break;
      bad = violations(path, start, *original);
      if (bad.empty()) {
        if (add_gait_ban(path, start, goal, *original))
          continue;
        RCLCPP_INFO(_logger,
                    "Swept raw fallback accepted after %d repair searches",
                    attempt);
        return path;
      }
      unsigned added = 0;
      for (const auto &p : bad) {
        unsigned x, y;
        if (search.worldToMap(p[0], p[1], x, y))
          for (int dx = -2; dx <= 2; ++dx)
            for (int dy = -2; dy <= 2; ++dy) {
              int xx = int(x) + dx, yy = int(y) + dy;
              if (xx >= 0 && yy >= 0 && xx < int(search.getSizeInCellsX()) &&
                  yy < int(search.getSizeInCellsY()) &&
                  search.getCost(xx, yy) < 253) {
                search.setCost(xx, yy, 253);
                ++added;
              }
            }
      }
      RCLCPP_WARN(_logger,
                  "Added %u reference-center exclusion cells; first invalid "
                  "%.4f %.4f %.4f",
                  added, bad.front()[0], bad.front()[1], bad.front()[2]);
    }
    throw nav2_core::PlannerException(
        "No fully checked swept-envelope path within bounded repair budget");
  }
};
} // namespace p2_nav_controller
PLUGINLIB_EXPORT_CLASS(p2_nav_controller::GaitFeasibleLattice,
                       nav2_core::GlobalPlanner)
