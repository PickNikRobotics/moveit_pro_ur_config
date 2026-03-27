// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include <compute_place_orientation/compute_place_orientation.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace compute_place_orientation
{
class ComputePlaceOrientationBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_pro::behaviors::registerBehavior<ComputePlaceOrientation>(factory, "ComputePlaceOrientation");
  }
};
}  // namespace compute_place_orientation

PLUGINLIB_EXPORT_CLASS(compute_place_orientation::ComputePlaceOrientationBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
