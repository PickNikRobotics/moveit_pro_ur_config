#include <ur_behaviors/get_highest_confidence_mask.hpp>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace
{
inline constexpr auto kDescriptionGetHighestConfidenceMask = R"(
    <p>Selects the mask with the highest confidence score from the output of a segmentation behavior.</p>
    <p>Takes parallel vectors of masks and confidence scores, returns the single best mask and its score.</p>
)";

constexpr auto kPortIDMasks2D = "masks2d";
constexpr auto kPortIDConfidenceScores = "confidence_scores";
constexpr auto kPortIDBestMask = "best_mask";
constexpr auto kPortIDBestScore = "best_score";
}  // namespace

namespace ur_behaviors
{

GetHighestConfidenceMask::GetHighestConfidenceMask(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
{
}

BT::PortsList GetHighestConfidenceMask::providedPorts()
{
  return {
    BT::InputPort<std::vector<moveit_studio_vision_msgs::msg::Mask2D>>(kPortIDMasks2D, "{masks2d}",
                                                                       "Vector of 2D masks from segmentation."),
    BT::InputPort<std::vector<double>>(kPortIDConfidenceScores, "{confidence_scores}",
                                       "Confidence scores parallel to masks2d."),
    BT::OutputPort<moveit_studio_vision_msgs::msg::Mask2D>(kPortIDBestMask, "{best_mask}",
                                                           "The mask with the highest confidence."),
    BT::OutputPort<double>(kPortIDBestScore, "{best_score}", "The highest confidence score."),
  };
}

BT::KeyValueVector GetHighestConfidenceMask::metadata()
{
  return { { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionGetHighestConfidenceMask },
           { moveit_pro::behaviors::kSubcategoryMetadataKey, "Perception - Segmentation" } };
}

BT::NodeStatus GetHighestConfidenceMask::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<std::vector<moveit_studio_vision_msgs::msg::Mask2D>>(kPortIDMasks2D),
      getInput<std::vector<double>>(kPortIDConfidenceScores));

  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                 ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [masks, scores] = ports.value();

  if (masks.empty())
  {
    shared_resources_->logger->publishFailureMessage(name(), "No masks provided — masks2d vector is empty.");
    return BT::NodeStatus::FAILURE;
  }

  if (masks.size() != scores.size())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Mask count (" + std::to_string(masks.size()) +
                                                                 ") does not match score count (" +
                                                                 std::to_string(scores.size()) + ").");
    return BT::NodeStatus::FAILURE;
  }

  // Find the index of the highest confidence score
  const auto best_it = std::max_element(scores.begin(), scores.end());
  const auto best_index = std::distance(scores.begin(), best_it);

  shared_resources_->logger->publishInfoMessage(name(), "Selected mask " + std::to_string(best_index) +
                                                            " with confidence " + std::to_string(*best_it) +
                                                            " out of " + std::to_string(masks.size()) + " masks.");

  setOutput(kPortIDBestMask, masks[best_index]);
  setOutput(kPortIDBestScore, *best_it);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace ur_behaviors
