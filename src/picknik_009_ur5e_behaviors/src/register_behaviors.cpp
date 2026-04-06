#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include <picknik_009_ur5e_behaviors/create_solid_primitive_from_shape.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace picknik_009_ur5e_behaviors
{
class Picknik009Ur5eBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(
      BT::BehaviorTreeFactory& factory,
      [[maybe_unused]] const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_pro::behaviors::registerBehavior<CreateSolidPrimitiveFromShape>(factory, "CreateSolidPrimitiveFromShape");
  }
};
}  // namespace picknik_009_ur5e_behaviors

PLUGINLIB_EXPORT_CLASS(picknik_009_ur5e_behaviors::Picknik009Ur5eBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);