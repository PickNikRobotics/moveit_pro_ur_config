// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include <get_graspable_grocery_objects_from_masks3d/compute_grocery_place_height.hpp>
#include <get_graspable_grocery_objects_from_masks3d/get_graspable_grocery_objects_from_masks3d.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace get_graspable_grocery_objects_from_masks3d
{
class GetGraspableGroceryObjectsFromMasks3DBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(
      BT::BehaviorTreeFactory& factory,
      [[maybe_unused]] const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_pro::behaviors::registerBehavior<ComputeGroceryPlaceHeight>(factory, "ComputeGroceryPlaceHeight",
                                                                       shared_resources);
    moveit_pro::behaviors::registerBehavior<GetGraspableGroceryObjectsFromMasks3D>(
        factory, "GetGraspableGroceryObjectsFromMasks3D", shared_resources);
  }
};
}  // namespace get_graspable_grocery_objects_from_masks3d

PLUGINLIB_EXPORT_CLASS(get_graspable_grocery_objects_from_masks3d::GetGraspableGroceryObjectsFromMasks3DBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
