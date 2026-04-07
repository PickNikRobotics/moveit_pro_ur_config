#include <picknik_009_ur5e_behaviors/read_yaml_list.hpp>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <rclcpp/rclcpp.hpp>

namespace picknik_009_ur5e_behaviors
{
ReadYamlList::ReadYamlList(const std::string& name, const BT::NodeConfiguration& config,
                           const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList ReadYamlList::providedPorts()
{
  return { BT::InputPort<std::string>("package_name", "Name of the ROS2 package containing the YAML file"),
           BT::InputPort<std::string>("relative_file_path", "Relative path from the package root to the YAML file"),
           BT::InputPort<std::string>("key1", "First key to traverse YAML structure (mandatory)"),
           BT::InputPort<std::string>("key2", "", "Second key to traverse YAML structure (optional)"),
           BT::InputPort<std::string>("key3", "", "Third key to traverse YAML structure (optional)"),
           BT::InputPort<std::string>("key4", "", "Fourth key to traverse YAML structure (optional)"),
           BT::InputPort<std::string>("key5", "", "Fifth key to traverse YAML structure (optional)"),
           BT::OutputPort<std::vector<std::string>>("value_list", "The extracted list as a vector of strings") };
}

BT::KeyValueVector ReadYamlList::metadata()
{
  return { { "description", "Reads a list from a YAML file as a vector of strings using nested keys" },
           { "subcategory", "YAML Operations" } };
}

BT::NodeStatus ReadYamlList::tick()
{
  // Get input port values
  auto package_name_result = getInput<std::string>("package_name");
  auto relative_file_path_result = getInput<std::string>("relative_file_path");
  auto key1_result = getInput<std::string>("key1");

  // Validate inputs
  if (!package_name_result)
  {
    RCLCPP_ERROR(shared_resources_->node->get_logger(), "Failed to get 'package_name' from input port: %s",
                 package_name_result.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!relative_file_path_result)
  {
    RCLCPP_ERROR(shared_resources_->node->get_logger(), "Failed to get 'relative_file_path' from input port: %s",
                 relative_file_path_result.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!key1_result)
  {
    RCLCPP_ERROR(shared_resources_->node->get_logger(), "Failed to get 'key1' from input port: %s",
                 key1_result.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const std::string& package_name = package_name_result.value();
  const std::string& relative_file_path = relative_file_path_result.value();

  // Build keys vector from individual key ports
  std::vector<std::string> keys;
  keys.push_back(key1_result.value());

  // Add optional keys if they are provided
  auto key2_result = getInput<std::string>("key2");
  if (key2_result && !key2_result.value().empty())
  {
    keys.push_back(key2_result.value());
  }

  auto key3_result = getInput<std::string>("key3");
  if (key3_result && !key3_result.value().empty())
  {
    keys.push_back(key3_result.value());
  }

  auto key4_result = getInput<std::string>("key4");
  if (key4_result && !key4_result.value().empty())
  {
    keys.push_back(key4_result.value());
  }

  auto key5_result = getInput<std::string>("key5");
  if (key5_result && !key5_result.value().empty())
  {
    keys.push_back(key5_result.value());
  }

  // Resolve the full file path
  std::string full_file_path;
  try
  {
    std::string package_share_dir = ament_index_cpp::get_package_share_directory(package_name);
    full_file_path = package_share_dir + "/" + relative_file_path;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(shared_resources_->node->get_logger(), "Failed to find package '%s': %s", package_name.c_str(),
                 e.what());
    return BT::NodeStatus::FAILURE;
  }

  // Check if file exists
  if (!std::filesystem::exists(full_file_path))
  {
    RCLCPP_ERROR(shared_resources_->node->get_logger(), "YAML file does not exist: %s", full_file_path.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Load the YAML file
  YAML::Node yaml_root;
  try
  {
    yaml_root = YAML::LoadFile(full_file_path);
  }
  catch (const YAML::Exception& e)
  {
    RCLCPP_ERROR(shared_resources_->node->get_logger(), "Failed to parse YAML file '%s': %s", full_file_path.c_str(),
                 e.what());
    return BT::NodeStatus::FAILURE;
  }

  // Traverse the YAML structure using the keys
  YAML::Node current_node = yaml_root;
  for (size_t i = 0; i < keys.size(); ++i)
  {
    const std::string& key = keys[i];

    if (!current_node.IsDefined())
    {
      RCLCPP_ERROR(shared_resources_->node->get_logger(), "YAML node is undefined at key '%s'", key.c_str());
      return BT::NodeStatus::FAILURE;
    }

    if (!current_node.IsMap())
    {
      RCLCPP_ERROR(shared_resources_->node->get_logger(), "YAML node at key '%s' is not a map", key.c_str());
      return BT::NodeStatus::FAILURE;
    }

    if (!current_node[key])
    {
      RCLCPP_ERROR(shared_resources_->node->get_logger(), "Key '%s' not found in YAML file", key.c_str());
      return BT::NodeStatus::FAILURE;
    }

    current_node = current_node[key];
  }

  // Extract the final value as a list
  std::vector<std::string> value_list;
  try
  {
    if (!current_node.IsSequence())
    {
      RCLCPP_ERROR(shared_resources_->node->get_logger(), "The final YAML node is not a sequence/list");
      return BT::NodeStatus::FAILURE;
    }

    // Convert all elements to strings
    for (std::size_t i = 0; i < current_node.size(); ++i)
    {
      const YAML::Node& element = current_node[i];

      if (element.IsScalar())
      {
        // Convert scalar values (string, int, float, bool) to string
        value_list.push_back(element.as<std::string>());
      }
      else
      {
        RCLCPP_WARN(shared_resources_->node->get_logger(), "List element at index %zu is not a scalar value, skipping",
                    i);
      }
    }

    if (value_list.empty())
    {
      RCLCPP_WARN(shared_resources_->node->get_logger(), "The list is empty or contains no scalar values");
    }
  }
  catch (const YAML::Exception& e)
  {
    RCLCPP_ERROR(shared_resources_->node->get_logger(), "Failed to convert YAML list to vector of strings: %s",
                 e.what());
    return BT::NodeStatus::FAILURE;
  }

  // Set the output port
  setOutput("value_list", value_list);

  RCLCPP_INFO(shared_resources_->node->get_logger(), "Successfully read list with %zu elements from YAML file: %s",
              value_list.size(), full_file_path.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace picknik_009_ur5e_behaviors
