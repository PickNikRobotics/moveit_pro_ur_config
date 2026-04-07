#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace picknik_009_ur5e_behaviors
{
/**
 * @brief Reads a list from a YAML file as a vector of strings given a package name, relative file path, and nested keys.
 * @details This behavior uses BT::SyncActionNode wrapped in SharedResourcesNode to read a YAML file
 * and extract a list value from a nested key structure, converting all elements to strings.
 */
class ReadYamlList : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  /**
   * @brief Constructor for ReadYamlList behavior.
   * @param name The name of a particular instance of this Behavior.
   * @param config Runtime configuration for this Behavior, including port mappings.
   * @param shared_resources Shared resources provided by the behavior context.
   */
  ReadYamlList(const std::string& name, const BT::NodeConfiguration& config,
               const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  /**
   * @brief Implementation of the required providedPorts() function.
   * @details Defines input and output ports for this behavior:
   * - Input: package_name (string) - Name of the ROS2 package
   * - Input: relative_file_path (string) - Relative path from package root to YAML file
   * - Input: key1 (string) - First key (mandatory)
   * - Input: key2 (string) - Second key (optional)
   * - Input: key3 (string) - Third key (optional)
   * - Input: key4 (string) - Fourth key (optional)
   * - Input: key5 (string) - Fifth key (optional)
   * - Output: value_list (vector<string>) - The extracted list as a vector of strings
   * @return A BT::PortsList containing the behavior's ports.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Implementation of the metadata() function for displaying behavior information.
   * @return A BT::KeyValueVector containing the behavior's metadata.
   */
  static BT::KeyValueVector metadata();

  /**
   * @brief Implementation of BT::SyncActionNode::tick().
   * @details Reads the YAML file, traverses the nested keys, and extracts the list as a vector of strings.
   * All list elements are converted to strings regardless of their original type.
   * This function completes synchronously.
   * @return BT::NodeStatus::SUCCESS if list was successfully read, FAILURE otherwise.
   */
  BT::NodeStatus tick() override;
};
}  // namespace picknik_009_ur5e_behaviors
