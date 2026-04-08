#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <moveit_studio_vision_msgs/msg/mask2_d.hpp>

#include <string>
#include <vector>

namespace ur_behaviors
{

/**
 * @brief Selects the mask with the highest confidence score from parallel mask and score vectors.
 *
 * @details
 * | Data Port Name       | Port Type | Object Type                                          |
 * | -------------------- | --------- | ---------------------------------------------------- |
 * | masks2d              | input     | std::vector<moveit_studio_vision_msgs::msg::Mask2D>  |
 * | confidence_scores    | input     | std::vector<double>                                  |
 * | best_mask            | output    | moveit_studio_vision_msgs::msg::Mask2D               |
 * | best_score           | output    | double                                               |
 */
class GetHighestConfidenceMask final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  GetHighestConfidenceMask(const std::string& name, const BT::NodeConfiguration& config,
                           const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace ur_behaviors
