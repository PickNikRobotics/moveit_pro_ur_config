#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include <filter_graspable_objects/filter_graspable_objects.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace filter_graspable_objects
{
class FilterGraspableObjectsBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(
      BT::BehaviorTreeFactory& factory,
      [[maybe_unused]] const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_pro::behaviors::registerBehavior<FilterGraspableObjects>(factory, "FilterGraspableObjects", shared_resources);
  }
};
}  // namespace filter_graspable_objects

PLUGINLIB_EXPORT_CLASS(filter_graspable_objects::FilterGraspableObjectsBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
