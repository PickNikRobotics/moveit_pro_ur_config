#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include <ur_behaviors/create_pose_stamped_grid.hpp>
#include <ur_behaviors/visualize_camera_frustum.hpp>
#include <ur_behaviors/visualize_placement_zones.hpp>
#include <ur_behaviors/get_highest_confidence_mask.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace ur_behaviors
{

class UrBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(
      BT::BehaviorTreeFactory& factory,
      const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_pro::behaviors::registerBehavior<CreatePoseStampedGrid>(
      factory, "CreatePoseStampedGrid", shared_resources);
    moveit_pro::behaviors::registerBehavior<VisualizeCameraFrustum>(
      factory, "VisualizeCameraFrustum", shared_resources);
    moveit_pro::behaviors::registerBehavior<VisualizePlacementZones>(
      factory, "VisualizePlacementZones", shared_resources);
    moveit_pro::behaviors::registerBehavior<GetHighestConfidenceMask>(
      factory, "GetHighestConfidenceMask", shared_resources);
  }
};

}  // namespace ur_behaviors

PLUGINLIB_EXPORT_CLASS(ur_behaviors::UrBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
