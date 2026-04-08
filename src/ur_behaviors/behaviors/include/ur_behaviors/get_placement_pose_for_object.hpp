#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

#include <memory>
#include <string>

namespace ur_behaviors
{

class GetPlacementPoseForObject final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  GetPlacementPoseForObject(const std::string& name, const BT::NodeConfiguration& config,
                            const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace ur_behaviors
