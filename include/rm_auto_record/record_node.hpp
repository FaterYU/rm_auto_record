#ifndef RM_AUTO_RECORD__RECORD_NODE_HPP_
#define RM_AUTO_RECORD__RECORD_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

namespace rm_auto_record
{
class RecordNode : public rclcpp::Node
{
public:
  explicit RecordNode(const rclcpp::NodeOptions & options);

private:
  void writeImage(std::shared_ptr<rclcpp::SerializedMessage> msg);

  void recordInit(std_msgs::msg::String::SharedPtr msg);

  // std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::shared_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr record_controller_sub_;

  enum RecordState {
    WAIT,
    INIT,
    RECORDING,
  };

  RecordState record_state_;
};

}  // namespace rm_auto_record

#endif  // RM_AUTO_RECORD__RECORD_NODE_HPP_