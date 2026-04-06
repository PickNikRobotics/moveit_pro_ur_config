#pragma once

#include <behaviortree_cpp/action_node.h>

#include <string>

namespace picknik_009_ur5e_behaviors
{

class GenerateSurfacePosesFromSolidPrimitive final : public BT::SyncActionNode
{
public:
  GenerateSurfacePosesFromSolidPrimitive(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace picknik_009_ur5e_behaviors