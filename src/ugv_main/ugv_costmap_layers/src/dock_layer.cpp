// Nav2 costmap layer that marks the charging dock as a fixed rectangle.
//
// The dock is given by its centre pose in `frame` (normally map) plus its size:
// `length` along the docking direction (pose yaw), `width` across it. The layer
// re-transforms it into the costmap's global frame on every update, so it also
// works in the odom-framed rolling local costmap. Put it before the inflation
// layer so the dock gets inflated.

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ugv_costmap_layers
{

class DockLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override
  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error("DockLayer: failed to lock node");
    }

    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("frame", rclcpp::ParameterValue(std::string("map")));
    declareParameter("pose", rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0}));
    declareParameter("length", rclcpp::ParameterValue(0.75));
    declareParameter("width", rclcpp::ParameterValue(0.5));
    declareParameter("cost", rclcpp::ParameterValue(static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE)));

    std::vector<double> pose;
    int cost;
    node->get_parameter(name_ + ".enabled", enabled_);
    node->get_parameter(name_ + ".frame", frame_);
    node->get_parameter(name_ + ".pose", pose);
    node->get_parameter(name_ + ".length", length_);
    node->get_parameter(name_ + ".width", width_);
    node->get_parameter(name_ + ".cost", cost);
    cost_ = static_cast<unsigned char>(std::clamp(cost, 1, 254));

    if (pose.size() != 3) {
      throw std::runtime_error("DockLayer: pose must be [x, y, yaw_deg]");
    }
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose[2] * M_PI / 180.0);
    dock_in_frame_ = tf2::Transform(q, tf2::Vector3(pose[0], pose[1], 0.0));

    // Always current: a missing TF must not block the planner (it waits on isCurrent()).
    current_ = true;
    global_frame_ = layered_costmap_->getGlobalFrameID();

    RCLCPP_INFO(
      logger_, "%s: dock %.2f x %.2f m at (%.3f, %.3f, %.1f deg) in %s, cost %d",
      name_.c_str(), length_, width_, pose[0], pose[1], pose[2], frame_.c_str(), cost_);
  }

  void updateBounds(
    double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
    double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    // The old area must be repainted too (map->odom drifts in the local costmap).
    Bounds bounds = last_bounds_;
    have_dock_ = enabled_ && lookupDock();

    Bounds now_bounds;
    if (have_dock_) {
      const double hl = length_ / 2.0, hw = width_ / 2.0;
      const std::array<std::pair<double, double>, 4> corners{{{hl, hw}, {hl, -hw}, {-hl, hw}, {-hl, -hw}}};
      for (const auto & c : corners) {
        const tf2::Vector3 p = dock_in_global_ * tf2::Vector3(c.first, c.second, 0.0);
        now_bounds.add(p.x(), p.y(), 0.0);
        bounds.add(p.x(), p.y(), 0.0);
      }
    }
    last_bounds_ = now_bounds;

    if (bounds.valid) {
      *min_x = std::min(*min_x, bounds.min_x);
      *min_y = std::min(*min_y, bounds.min_y);
      *max_x = std::max(*max_x, bounds.max_x);
      *max_y = std::max(*max_y, bounds.max_y);
    }
  }

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override
  {
    if (!have_dock_ || !last_bounds_.valid) {
      return;
    }
    const double res = master_grid.getResolution();
    int i0, j0, i1, j1;
    master_grid.worldToMapNoBounds(last_bounds_.min_x - res, last_bounds_.min_y - res, i0, j0);
    master_grid.worldToMapNoBounds(last_bounds_.max_x + res, last_bounds_.max_y + res, i1, j1);

    const tf2::Transform global_to_dock = dock_in_global_.inverse();
    const double hl = length_ / 2.0, hw = width_ / 2.0;

    for (int j = std::max(j0, min_j); j <= std::min(j1, max_j - 1); ++j) {
      for (int i = std::max(i0, min_i); i <= std::min(i1, max_i - 1); ++i) {
        double wx, wy;
        master_grid.mapToWorld(i, j, wx, wy);
        const tf2::Vector3 d = global_to_dock * tf2::Vector3(wx, wy, 0.0);
        if (std::abs(d.x()) > hl || std::abs(d.y()) > hw) {
          continue;
        }
        const unsigned char old = master_grid.getCost(i, j);
        if (old == nav2_costmap_2d::NO_INFORMATION || old < cost_) {
          master_grid.setCost(i, j, cost_);
        }
      }
    }
  }

  void reset() override {}

  bool isClearable() override {return false;}

private:
  struct Bounds
  {
    double min_x, min_y, max_x, max_y;
    bool valid = false;

    void add(double x, double y, double pad)
    {
      if (!valid) {
        min_x = x - pad; min_y = y - pad; max_x = x + pad; max_y = y + pad;
        valid = true;
        return;
      }
      min_x = std::min(min_x, x - pad);
      min_y = std::min(min_y, y - pad);
      max_x = std::max(max_x, x + pad);
      max_y = std::max(max_y, y + pad);
    }
  };

  bool lookupDock()
  {
    if (frame_ == global_frame_) {
      dock_in_global_ = dock_in_frame_;
      return true;
    }
    try {
      const auto t = tf_->lookupTransform(global_frame_, frame_, tf2::TimePointZero);
      tf2::Transform tf;
      tf2::fromMsg(t.transform, tf);
      dock_in_global_ = tf * dock_in_frame_;
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 10000, "%s: %s -> %s TF failed: %s",
        name_.c_str(), frame_.c_str(), global_frame_.c_str(), e.what());
      return false;
    }
  }

  std::string frame_;
  std::string global_frame_;
  tf2::Transform dock_in_frame_;
  tf2::Transform dock_in_global_;
  bool have_dock_{false};
  Bounds last_bounds_;

  double length_{0.75};
  double width_{0.5};
  unsigned char cost_{nav2_costmap_2d::LETHAL_OBSTACLE};
};

}  // namespace ugv_costmap_layers

PLUGINLIB_EXPORT_CLASS(ugv_costmap_layers::DockLayer, nav2_costmap_2d::Layer)
