#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <moveit_studio_vision_msgs/msg/graspable_object.hpp>
#include <rclcpp/publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace filter_graspable_objects
{

/**
 * @brief Filters a vector of GraspableObjects by maximum linear dimension and optionally by volume.
 *        Results are sorted by centroid height (highest Z first).
 *        Publishes blue transparent markers for rejected objects and green for accepted.
 */
class FilterGraspableObjects final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  FilterGraspableObjects(const std::string& name, const BT::NodeConfiguration& config,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace filter_graspable_objects
