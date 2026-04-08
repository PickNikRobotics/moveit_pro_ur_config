// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/node.hpp>

/**
 * @brief This test makes sure that the Behaviors provided in this package can be successfully registered and
 * instantiated by the behavior tree factory.
 */
TEST(BehaviorTests, LoadAndInstantiate)
{
  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> class_loader(
      "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");

  auto node = std::make_shared<rclcpp::Node>("BehaviorTests");
  auto shared_resources = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node);

  BT::BehaviorTreeFactory factory;
  {
    auto plugin_instance = class_loader.createUniqueInstance("generate_shape_urdf::GenerateShapeUrdfBehaviorsLoader");
    ASSERT_NO_THROW(plugin_instance->registerBehaviors(factory, shared_resources));
  }

  EXPECT_NO_THROW((void)factory.instantiateTreeNode("test_behavior_name", "GenerateShapeUrdf", BT::NodeConfiguration()));
}

TEST(BehaviorTests, GenerateBoxUrdf)
{
  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> class_loader(
      "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");

  auto node = std::make_shared<rclcpp::Node>("BehaviorTests");
  auto shared_resources = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node);

  BT::BehaviorTreeFactory factory;
  {
    auto plugin_instance = class_loader.createUniqueInstance("generate_shape_urdf::GenerateShapeUrdfBehaviorsLoader");
    plugin_instance->registerBehaviors(factory, shared_resources);
  }

  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  config.blackboard->set("shape_type", std::string("box"));
  config.blackboard->set("dimensions", std::vector<double>{ 0.1, 0.2, 0.3 });
  config.input_ports["shape_type"] = "{shape_type}";
  config.input_ports["dimensions"] = "{dimensions}";
  config.output_ports["urdf_string"] = "{urdf_string}";

  auto behavior = factory.instantiateTreeNode("test", "GenerateShapeUrdf", config);
  ASSERT_EQ(behavior->executeTick(), BT::NodeStatus::SUCCESS);

  const auto urdf_string = config.blackboard->get<std::string>("urdf_string");
  EXPECT_NE(urdf_string.find("<robot name=\"generated_box\">"), std::string::npos);
  EXPECT_NE(urdf_string.find("<box size=\"0.1 0.2 0.3\"/>"), std::string::npos);
}

TEST(BehaviorTests, GenerateSphereUrdf)
{
  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> class_loader(
      "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");

  auto node = std::make_shared<rclcpp::Node>("BehaviorTests");
  auto shared_resources = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node);

  BT::BehaviorTreeFactory factory;
  {
    auto plugin_instance = class_loader.createUniqueInstance("generate_shape_urdf::GenerateShapeUrdfBehaviorsLoader");
    plugin_instance->registerBehaviors(factory, shared_resources);
  }

  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  config.blackboard->set("shape_type", std::string("sphere"));
  config.blackboard->set("dimensions", std::vector<double>{ 0.05 });
  config.input_ports["shape_type"] = "{shape_type}";
  config.input_ports["dimensions"] = "{dimensions}";
  config.output_ports["urdf_string"] = "{urdf_string}";

  auto behavior = factory.instantiateTreeNode("test", "GenerateShapeUrdf", config);
  ASSERT_EQ(behavior->executeTick(), BT::NodeStatus::SUCCESS);

  const auto urdf_string = config.blackboard->get<std::string>("urdf_string");
  EXPECT_NE(urdf_string.find("<robot name=\"generated_sphere\">"), std::string::npos);
  EXPECT_NE(urdf_string.find("<sphere radius=\"0.05\"/>"), std::string::npos);
}

TEST(BehaviorTests, GenerateCylinderUrdf)
{
  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> class_loader(
      "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");

  auto node = std::make_shared<rclcpp::Node>("BehaviorTests");
  auto shared_resources = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node);

  BT::BehaviorTreeFactory factory;
  {
    auto plugin_instance = class_loader.createUniqueInstance("generate_shape_urdf::GenerateShapeUrdfBehaviorsLoader");
    plugin_instance->registerBehaviors(factory, shared_resources);
  }

  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  config.blackboard->set("shape_type", std::string("cylinder"));
  config.blackboard->set("dimensions", std::vector<double>{ 0.05, 0.2 });
  config.input_ports["shape_type"] = "{shape_type}";
  config.input_ports["dimensions"] = "{dimensions}";
  config.output_ports["urdf_string"] = "{urdf_string}";

  auto behavior = factory.instantiateTreeNode("test", "GenerateShapeUrdf", config);
  ASSERT_EQ(behavior->executeTick(), BT::NodeStatus::SUCCESS);

  const auto urdf_string = config.blackboard->get<std::string>("urdf_string");
  EXPECT_NE(urdf_string.find("<robot name=\"generated_cylinder\">"), std::string::npos);
  EXPECT_NE(urdf_string.find("<cylinder radius=\"0.05\" length=\"0.2\"/>"), std::string::npos);
}

TEST(BehaviorTests, InvalidShapeTypeFails)
{
  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> class_loader(
      "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");

  auto node = std::make_shared<rclcpp::Node>("BehaviorTests");
  auto shared_resources = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node);

  BT::BehaviorTreeFactory factory;
  {
    auto plugin_instance = class_loader.createUniqueInstance("generate_shape_urdf::GenerateShapeUrdfBehaviorsLoader");
    plugin_instance->registerBehaviors(factory, shared_resources);
  }

  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  config.blackboard->set("shape_type", std::string("triangle"));
  config.blackboard->set("dimensions", std::vector<double>{ 0.1 });
  config.input_ports["shape_type"] = "{shape_type}";
  config.input_ports["dimensions"] = "{dimensions}";
  config.output_ports["urdf_string"] = "{urdf_string}";

  auto behavior = factory.instantiateTreeNode("test", "GenerateShapeUrdf", config);
  EXPECT_EQ(behavior->executeTick(), BT::NodeStatus::FAILURE);
}

TEST(BehaviorTests, WrongDimensionCountFails)
{
  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> class_loader(
      "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");

  auto node = std::make_shared<rclcpp::Node>("BehaviorTests");
  auto shared_resources = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node);

  BT::BehaviorTreeFactory factory;
  {
    auto plugin_instance = class_loader.createUniqueInstance("generate_shape_urdf::GenerateShapeUrdfBehaviorsLoader");
    plugin_instance->registerBehaviors(factory, shared_resources);
  }

  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  config.blackboard->set("shape_type", std::string("box"));
  config.blackboard->set("dimensions", std::vector<double>{ 0.1, 0.2 });  // box needs 3
  config.input_ports["shape_type"] = "{shape_type}";
  config.input_ports["dimensions"] = "{dimensions}";
  config.output_ports["urdf_string"] = "{urdf_string}";

  auto behavior = factory.instantiateTreeNode("test", "GenerateShapeUrdf", config);
  EXPECT_EQ(behavior->executeTick(), BT::NodeStatus::FAILURE);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
