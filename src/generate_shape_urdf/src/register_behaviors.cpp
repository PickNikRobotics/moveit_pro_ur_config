// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include <generate_shape_urdf/generate_shape_urdf.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace generate_shape_urdf
{
class GenerateShapeUrdfBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_pro::behaviors::registerBehavior<GenerateShapeUrdf>(factory, "GenerateShapeUrdf");
  }
};
}  // namespace generate_shape_urdf

PLUGINLIB_EXPORT_CLASS(generate_shape_urdf::GenerateShapeUrdfBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
