#include <cstdint>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"

namespace ugv_nav_bt
{

// Ticks its child (a straight calibration drive) once per new non-zero id on
// request_topic, and always ends in SUCCESS so navigation simply continues.
class HeadingCalibration : public BT::DecoratorNode
{
public:
  HeadingCalibration(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::DecoratorNode(name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    std::string topic = "/uwb/heading_calibration_request";
    getInput("request_topic", topic);
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    sub_ = node_->create_subscription<std_msgs::msg::UInt32>(
      topic, rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::UInt32::SharedPtr msg) {latest_request_ = msg->data;},
      options);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "request_topic", "/uwb/heading_calibration_request",
        "Calibration request id topic (0 = no request)")
    };
  }

private:
  BT::NodeStatus tick() override
  {
    executor_.spin_some();

    if (status() != BT::NodeStatus::RUNNING) {
      if (latest_request_ == 0 || latest_request_ == handled_request_) {
        return BT::NodeStatus::SUCCESS;
      }
      handled_request_ = latest_request_;
      RCLCPP_INFO(
        node_->get_logger(), "HeadingCalibration: straight calibration drive #%u",
        handled_request_);
    }

    const BT::NodeStatus child_status = child_node_->executeTick();
    if (child_status == BT::NodeStatus::RUNNING) {
      return BT::NodeStatus::RUNNING;
    }
    // A failed drive (e.g. obstacle ahead) must not abort navigation.
    haltChild();
    return BT::NodeStatus::SUCCESS;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr sub_;
  uint32_t latest_request_{0};
  uint32_t handled_request_{0};
};

}  // namespace ugv_nav_bt

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ugv_nav_bt::HeadingCalibration>("HeadingCalibration");
}
