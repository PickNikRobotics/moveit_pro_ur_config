#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <rclcpp/publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>

namespace ur_behaviors
{

class VisualizePlacementZones final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  VisualizePlacementZones(const std::string& name, const BT::NodeConfiguration& config,
                          const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> publisher_;
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> text_publisher_;
};

}  // namespace ur_behaviors
