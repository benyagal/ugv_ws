// Nav2 costmap layer for live chickens reported by the overhead camera system.
//
// Input: geometry_msgs/PoseArray (one pose per chicken, any frame with a TF to
// the costmap's global frame). Only chickens within max_range of the robot are
// marked, each as a disk of `radius`. The layer is rebuilt from the latest
// message on every update, so a chicken that moved leaves no trail. Data older
// than `timeout` (receive time, the camera Jetson's clock may differ) is dropped.

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
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

class ChickenLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override
  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error("ChickenLayer: failed to lock node");
    }

    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("topic", rclcpp::ParameterValue(std::string("/overhead/chickens")));
    declareParameter("radius", rclcpp::ParameterValue(0.15));
    declareParameter("cost", rclcpp::ParameterValue(static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE)));
    declareParameter("max_range", rclcpp::ParameterValue(4.0));
    declareParameter("min_range", rclcpp::ParameterValue(0.2));
    declareParameter("timeout", rclcpp::ParameterValue(1.0));

    std::string topic;
    int cost;
    node->get_parameter(name_ + ".enabled", enabled_);
    node->get_parameter(name_ + ".topic", topic);
    node->get_parameter(name_ + ".radius", radius_);
    node->get_parameter(name_ + ".cost", cost);
    node->get_parameter(name_ + ".max_range", max_range_);
    node->get_parameter(name_ + ".min_range", min_range_);
    node->get_parameter(name_ + ".timeout", timeout_);
    cost_ = static_cast<unsigned char>(std::clamp(cost, 1, 254));

    // Always current: a missing camera system must not block the planner (it waits on isCurrent()).
    current_ = true;
    global_frame_ = layered_costmap_->getGlobalFrameID();

    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
      topic, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_msg_ = msg;
        last_rx_ = clock_->now();
      },
      options);

    RCLCPP_INFO(
      logger_, "%s: chickens from %s, radius %.2f m, cost %d, range %.1f-%.1f m",
      name_.c_str(), topic.c_str(), radius_, cost_, min_range_, max_range_);
  }

  void updateBounds(
    double robot_x, double robot_y, double /*robot_yaw*/,
    double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    // Old marks must be repainted (cleared) too, so report last cycle's area as well.
    Bounds bounds = last_bounds_;
    points_.clear();

    if (enabled_) {
      collectPoints(robot_x, robot_y);
    }

    Bounds now_bounds;
    for (const auto & p : points_) {
      now_bounds.add(p.first, p.second, radius_);
      bounds.add(p.first, p.second, radius_);
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
    if (!enabled_ || points_.empty()) {
      return;
    }
    const double res = master_grid.getResolution();
    const int r_cells = static_cast<int>(std::ceil(radius_ / res));
    const double r2 = radius_ * radius_;

    for (const auto & p : points_) {
      int cx, cy;
      master_grid.worldToMapNoBounds(p.first, p.second, cx, cy);
      for (int j = std::max(cy - r_cells, min_j); j <= std::min(cy + r_cells, max_j - 1); ++j) {
        for (int i = std::max(cx - r_cells, min_i); i <= std::min(cx + r_cells, max_i - 1); ++i) {
          const double dx = (i - cx) * res;
          const double dy = (j - cy) * res;
          if (dx * dx + dy * dy > r2) {
            continue;
          }
          const unsigned char old = master_grid.getCost(i, j);
          if (old == nav2_costmap_2d::NO_INFORMATION || old < cost_) {
            master_grid.setCost(i, j, cost_);
          }
        }
      }
    }
  }

  void reset() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_msg_.reset();
    points_.clear();
  }

  bool isClearable() override {return true;}

private:
  void collectPoints(double robot_x, double robot_y)
  {
    geometry_msgs::msg::PoseArray::ConstSharedPtr msg;
    rclcpp::Time rx;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      msg = last_msg_;
      rx = last_rx_;
    }
    if (!msg) {
      return;
    }

    const double age = (clock_->now() - rx).seconds();
    if (age > timeout_) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 10000, "%s: no chicken data for %.1f s, ignoring the last list",
        name_.c_str(), age);
      return;
    }

    tf2::Transform tf;
    try {
      const auto t = tf_->lookupTransform(global_frame_, msg->header.frame_id, tf2::TimePointZero);
      tf2::fromMsg(t.transform, tf);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 10000, "%s: %s -> %s TF failed: %s",
        name_.c_str(), msg->header.frame_id.c_str(), global_frame_.c_str(), e.what());
      return;
    }

    for (const auto & pose : msg->poses) {
      const tf2::Vector3 p = tf * tf2::Vector3(pose.position.x, pose.position.y, 0.0);
      const double d = std::hypot(p.x() - robot_x, p.y() - robot_y);
      // min_range: a chicken reported under the robot (position error) would only lock it in place.
      if (d <= max_range_ && d >= min_range_) {
        points_.emplace_back(p.x(), p.y());
      }
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
  std::mutex mutex_;
  geometry_msgs::msg::PoseArray::ConstSharedPtr last_msg_;
  rclcpp::Time last_rx_;

  std::string global_frame_;
  std::vector<std::pair<double, double>> points_;
  Bounds last_bounds_;

  double radius_{0.15};
  double max_range_{4.0};
  double min_range_{0.2};
  double timeout_{1.0};
  unsigned char cost_{nav2_costmap_2d::LETHAL_OBSTACLE};
};

}  // namespace ugv_costmap_layers

PLUGINLIB_EXPORT_CLASS(ugv_costmap_layers::ChickenLayer, nav2_costmap_2d::Layer)
