// Copyright 2026 AGT Runtime
// SPDX-License-Identifier: Apache-2.0

// Standard PointCloud2 output for the MID360 LiDAR.
//
// Converts the Livox CustomMsg stream (with per-point offset_time / line / tag
// preserved for FAST-LIVO2) into the standard `sensor_msgs/PointCloud2` topic
// consumed by RTAB-Map:
//
//   /agt/sensors/lidar/custom_filtered  (CustomMsg, self-filtered)
//                 |
//                 v
//   livox_custom_to_pointcloud2
//                 |
//                 v
//   /agt/sensors/lidar/points           (sensor_msgs/PointCloud2, x y z intensity)
//
// The conversion keeps every point (x, y, z, reflectivity) in the input order;
// self-filtering of the robot body stays in the upstream self-filter node so
// both FAST-LIVO2 and RTAB-Map consume the same filtered geometry.

#include <cstring>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace agt_sensor_adapters
{

class LivoxCustomToPointCloud2 : public rclcpp::Node
{
public:
  LivoxCustomToPointCloud2()
  : Node("agt_livox_custom_to_pointcloud2")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/agt/sensors/lidar/custom_filtered");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/agt/sensors/lidar/points");
    frame_id_ = declare_parameter<std::string>("frame_id", "");
    queue_depth_ = declare_parameter<int>("queue_depth", 10);

    // Best-effort input accepts both live sensor and rosbag QoS.
    const auto input_qos = rclcpp::SensorDataQoS().keep_last(queue_depth_);
    // Reliable output so default (reliable) consumers such as RTAB-Map can
    // subscribe; best-effort consumers are also compatible.
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(queue_depth_)).reliable();

    subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_topic_, input_qos,
      std::bind(&LivoxCustomToPointCloud2::convert, this, std::placeholders::_1));
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, output_qos);

    RCLCPP_INFO(
      get_logger(), "converting %s -> %s (frame_id='%s')",
      input_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str());
  }

private:
  void convert(const livox_ros_driver2::msg::CustomMsg::SharedPtr message)
  {
    const auto & points = message->points;
    const size_t declared_count = message->point_num;
    const size_t count = std::min(declared_count, points.size());
    if (count == 0) {
      return;
    }
    if (declared_count != points.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Livox point_num (%zu) differs from points.size() (%zu); using %zu points",
        declared_count, points.size(), count);
    }

    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud->header = message->header;
    if (frame_id_.empty()) {
      // Keep the driver frame (livox_frame) by default.
      cloud->header.frame_id = message->header.frame_id;
    } else {
      cloud->header.frame_id = frame_id_;
    }
    // Stamped with the base time plus the last point's offset: the same
    // convention as the FAST-LIVO2 adapter's per-frame timestamp.
    const uint64_t timebase = message->timebase;
    const uint64_t last_offset = points.empty() ? 0u : points.back().offset_time;
    if (timebase != 0u) {
      cloud->header.stamp = rclcpp::Time(timebase + last_offset);
    }

    cloud->height = 1;
    cloud->width = static_cast<uint32_t>(count);
    cloud->is_dense = true;
    cloud->is_bigendian = false;

    // Build fields manually: Humble's setPointCloud2FieldsByString only
    // supports "xyz"/"rgb"/"rgba" and would reject "intensity".
    auto make_field = [](const char * name, uint32_t offset, uint8_t datatype) {
      sensor_msgs::msg::PointField field;
      field.name = name;
      field.offset = offset;
      field.datatype = datatype;
      field.count = 1;
      return field;
    };
    cloud->fields = {
      make_field("x", 0, sensor_msgs::msg::PointField::FLOAT32),
      make_field("y", 4, sensor_msgs::msg::PointField::FLOAT32),
      make_field("z", 8, sensor_msgs::msg::PointField::FLOAT32),
      make_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32),
    };
    cloud->point_step = 16;
    cloud->row_step = cloud->width * cloud->point_step;
    cloud->data.resize(cloud->height * cloud->row_step);

    sensor_msgs::PointCloud2Iterator<float> it_x(*cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(*cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(*cloud, "z");
    // intensity = reflectivity (0..255) kept as float32.
    sensor_msgs::PointCloud2Iterator<float> it_i(*cloud, "intensity");
    for (size_t index = 0; index < count; ++index) {
      *it_x = points[index].x;
      *it_y = points[index].y;
      *it_z = points[index].z;
      *it_i = static_cast<float>(points[index].reflectivity);
      ++it_x; ++it_y; ++it_z; ++it_i;
    }
    publisher_->publish(std::move(cloud));
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  int queue_depth_{10};
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

}  // namespace agt_sensor_adapters

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agt_sensor_adapters::LivoxCustomToPointCloud2>());
  rclcpp::shutdown();
  return 0;
}
