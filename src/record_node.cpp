#include "rm_auto_record/record_node.hpp"

namespace rm_auto_record
{

RecordNode::RecordNode(const rclcpp::NodeOptions & options) : Node("rm_auto_record_node", options)
{
  record_state_ = WAIT;

  record_controller_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/record_controller", 10, std::bind(&RecordNode::recordInit, this, std::placeholders::_1));
}

void RecordNode::recordInit(std_msgs::msg::Int16::SharedPtr msg)
{
  if (msg->data == 1 && record_state_ == WAIT) {
    record_state_ = INIT;
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    now_c += 8 * 3600;  // UTC+8
    auto yyyy_mm_dd_hh_mm_ss = std::localtime(&now_c);
    std::string bag_name = "record_" + std::to_string(yyyy_mm_dd_hh_mm_ss->tm_year + 1900) + "_" +
                           std::to_string(yyyy_mm_dd_hh_mm_ss->tm_mon + 1) + "_" +
                           std::to_string(yyyy_mm_dd_hh_mm_ss->tm_mday) + "_" +
                           std::to_string(yyyy_mm_dd_hh_mm_ss->tm_hour) + "_" +
                           std::to_string(yyyy_mm_dd_hh_mm_ss->tm_min) + "_" +
                           std::to_string(yyyy_mm_dd_hh_mm_ss->tm_sec);

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = "/ros_ws/" + bag_name;
    storage_options.storage_id = "sqlite3";

    storage_options.max_bagfile_duration = 30;  // 30s
    const rosbag2_cpp::ConverterOptions converter_options(
      {rmw_get_serialization_format(), rmw_get_serialization_format()});

    writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
    writer_->open(storage_options, converter_options);

    writer_->create_topic(
      {"/detector/result_img/compressed", "sensor_msgs/msg/CompressedImage",
       rmw_get_serialization_format(), ""});
    RCLCPP_INFO(get_logger(), "Start recording.");

    image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      "/detector/result_img/compressed", 10,
      std::bind(&RecordNode::writeImage, this, std::placeholders::_1));
  }
  if (msg->data == 0 && record_state_ != WAIT) {
    record_state_ = WAIT;
    writer_->close();
    RCLCPP_INFO(get_logger(), "Stop recording.");
  }
}

void RecordNode::writeImage(std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  if (record_state_ == WAIT) {
    return;
  }
  record_state_ = RECORDING;
  auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

  bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
    new rcutils_uint8_array_t, [this](rcutils_uint8_array_t * msg) {
      auto fini_return = rcutils_uint8_array_fini(msg);
      delete msg;
      if (fini_return != RCUTILS_RET_OK) {
        RCLCPP_ERROR(
          get_logger(), "Failed to destroy serialized message %s", rcutils_get_error_string().str);
      }
    });
  *bag_message->serialized_data = msg->release_rcl_serialized_message();

  bag_message->topic_name = "/detector/result_img/compressed";
  if (rcutils_system_time_now(&bag_message->time_stamp) != RCUTILS_RET_OK) {
    RCLCPP_ERROR(get_logger(), "Error getting current time: %s", rcutils_get_error_string().str);
  }

  writer_->write(bag_message);
  RCLCPP_DEBUG(get_logger(), "Image written to bag.");
}

}  // namespace rm_auto_record

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_record::RecordNode)