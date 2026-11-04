#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace p2_nav_controller {
// Global dynamic observations have a finite memory. Expiration removes only
// this layer's mark; StaticLayer and the map-boundary filter still apply.
// Local ObstacleLayer retains its existing behavior.
class TemporalObstacleLayer : public nav2_costmap_2d::ObstacleLayer {
  std::vector<int64_t> seen_;
  double lifetime_=3., source_age_=.3;
 public:
  void onInitialize() override {
    ObstacleLayer::onInitialize();
    auto node=node_.lock();
    declareParameter("observation_keep_time",rclcpp::ParameterValue(3.));
    declareParameter("expiration_source_max_age",rclcpp::ParameterValue(.3));
    node->get_parameter(name_+".observation_keep_time",lifetime_);
    node->get_parameter(name_+".expiration_source_max_age",source_age_);
    if(!std::isfinite(lifetime_)||lifetime_<=0.||!std::isfinite(source_age_)||source_age_<=0.)
      throw std::invalid_argument("Invalid global dynamic-observation lifetime");
    if(rolling_window_)throw std::invalid_argument("TemporalObstacleLayer is for the fixed global costmap");
  }
  void reset() override {ObstacleLayer::reset();seen_.clear();}
  void matchSize() override {ObstacleLayer::matchSize();seen_.clear();}
  void updateBounds(double x,double y,double yaw,double *min_x,double *min_y,double *max_x,double *max_y) override {
    ObstacleLayer::updateBounds(x,y,yaw,min_x,min_y,max_x,max_y);
    if(!enabled_)return;
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());
    const auto node=node_.lock();if(!node)return;
    const int64_t now=node->now().nanoseconds();
    const size_t size=static_cast<size_t>(getSizeInCellsX())*getSizeInCellsY();
    if(seen_.size()!=size)seen_.assign(size,0);
    std::vector<nav2_costmap_2d::Observation> observations;
    getMarkingObservations(observations);
    bool fresh=false;
    for(const auto &observation:observations) {
      const int64_t stamp=rclcpp::Time(observation.cloud_->header.stamp).nanoseconds();
      const double age=static_cast<double>(now-stamp)/1e9;
      // The stock layer can mark its latest buffered cloud even when delayed.
      // Register that mark using its ORIGINAL producer time. Skipping late
      // samples here leaves stock-layer marks without an expiry timestamp and
      // can permanently close a previously traversed corridor. Replaying the
      // same old stamp cannot make a mark newer; only a genuinely fresh source
      // below authorizes expiry. Do not treat a late cloud as current health.
      if(stamp<=0||age<-.05)continue;
      fresh=fresh||age<=source_age_;
      sensor_msgs::PointCloud2ConstIterator<float> px(*observation.cloud_,"x"),py(*observation.cloud_,"y"),pz(*observation.cloud_,"z");
      for(;px!=px.end();++px,++py,++pz) {
        if(!std::isfinite(*px)||!std::isfinite(*py)||!std::isfinite(*pz)||*pz<min_obstacle_height_||*pz>max_obstacle_height_)continue;
        const double dx=*px-observation.origin_.x,dy=*py-observation.origin_.y,dz=*pz-observation.origin_.z;
        const double distance=dx*dx+dy*dy+dz*dz;
        if(distance>=observation.obstacle_max_range_*observation.obstacle_max_range_||distance<observation.obstacle_min_range_*observation.obstacle_min_range_)continue;
        unsigned mx,my;if(!worldToMap(*px,*py,mx,my))continue;
        const auto index=getIndex(mx,my);
        if(costmap_[index]==nav2_costmap_2d::LETHAL_OBSTACLE)seen_[index]=std::max(seen_[index],stamp);
      }
    }
    // An offline sensor cannot erase remembered obstacles as its clock ages.
    if(!fresh)return;
    for(size_t i=0;i<size;++i)if(seen_[i]>0) {
      if(costmap_[i]!=nav2_costmap_2d::LETHAL_OBSTACLE){seen_[i]=0;continue;}
      if(now-seen_[i]<=static_cast<int64_t>(lifetime_*1e9))continue;
      costmap_[i]=nav2_costmap_2d::NO_INFORMATION;seen_[i]=0;
      double wx,wy;mapToWorld(i%getSizeInCellsX(),i/getSizeInCellsX(),wx,wy);
      // Include expired cells in this update so the master restores its static
      // value even when the robot has left that part of the map.
      *min_x=std::min(*min_x,wx-getResolution()/2.);*max_x=std::max(*max_x,wx+getResolution()/2.);
      *min_y=std::min(*min_y,wy-getResolution()/2.);*max_y=std::max(*max_y,wy+getResolution()/2.);
    }
  }
};
}
PLUGINLIB_EXPORT_CLASS(p2_nav_controller::TemporalObstacleLayer,nav2_costmap_2d::Layer)
