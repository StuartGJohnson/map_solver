#include "map_solver/scan_bag_converter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <utility>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>

namespace map_solver
{
namespace
{

template<typename MsgT>
MsgT deserializeMessage(
  const rosbag2_storage::SerializedBagMessage & bag_msg,
  rclcpp::Serialization<MsgT> & serializer)
{
  const auto & src = bag_msg.serialized_data;
  if (!src || !src->buffer || src->buffer_length == 0U) {
    throw std::runtime_error("empty serialized bag message");
  }

  rclcpp::SerializedMessage serialized(src->buffer_length);
  auto & dst = serialized.get_rcl_serialized_message();
  std::memcpy(dst.buffer, src->buffer, src->buffer_length);
  dst.buffer_length = src->buffer_length;

  MsgT out;
  serializer.deserialize_message(&serialized, &out);
  return out;
}

std::filesystem::path firstBagPath(const std::filesystem::path & waypoint_directory)
{
  if (!std::filesystem::exists(waypoint_directory)) {
    throw std::runtime_error("waypoint directory does not exist: " + waypoint_directory.string());
  }

  std::vector<std::filesystem::path> candidates;
  for (const auto & entry : std::filesystem::directory_iterator(waypoint_directory)) {
    if (entry.is_regular_file() && entry.path().extension() == ".mcap") {
      candidates.push_back(entry.path());
    }
  }
  std::sort(candidates.begin(), candidates.end());
  if (candidates.empty()) {
    throw std::runtime_error("no .mcap file found in: " + waypoint_directory.string());
  }
  return candidates.front();
}

}  // namespace

ScanBagConverter::ScanBagConverter(ScanBagConversionOptions options)
: options_(std::move(options))
{
  options_.scan_stride = std::max(options_.scan_stride, 1);
  options_.sigma_r = std::max(options_.sigma_r, 1.0e-6);
  options_.sigma_theta = std::max(options_.sigma_theta, 0.0);
}

ScanBundle ScanBagConverter::readWaypointDirectory(
  const std::filesystem::path & waypoint_directory) const
{
  const auto bag_path = firstBagPath(waypoint_directory);

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path.string();
  storage_options.storage_id = options_.storage_id;

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  rosbag2_cpp::Reader reader;
  reader.open(storage_options, converter_options);

  rclcpp::Serialization<sensor_msgs::msg::LaserScan> serializer;
  ScanBundle bundle;
  bundle.waypoint_directory = waypoint_directory;
  bundle.bag_path = bag_path;
  bundle.waypoint_name = waypoint_directory.filename().string();
  int topic_scan_index = 0;
  while (reader.has_next()) {
    const auto bag_msg = reader.read_next();
    if (!bag_msg || bag_msg->topic_name != options_.topic) {
      continue;
    }
    if ((topic_scan_index++ % options_.scan_stride) != 0) {
      continue;
    }

    const auto scan = deserializeMessage(*bag_msg, serializer);
    const auto points = convertLaserScan(scan);
    bundle.scan_count += 1U;
    bundle.finite_range_count += points.endpoints.size();
    bundle.points.endpoints.insert(
      bundle.points.endpoints.end(), points.endpoints.begin(), points.endpoints.end());
    bundle.points.ranges.insert(
      bundle.points.ranges.end(), points.ranges.begin(), points.ranges.end());
    bundle.points.variances.insert(
      bundle.points.variances.end(), points.variances.begin(), points.variances.end());

    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const double angle = static_cast<double>(scan.angle_min) +
        static_cast<double>(i) * static_cast<double>(scan.angle_increment);
      const double measured_range = static_cast<double>(scan.ranges[i]);
      const double max_range = static_cast<double>(scan.range_max);
      const bool hit = std::isfinite(measured_range) &&
        measured_range >= static_cast<double>(scan.range_min) &&
        measured_range <= max_range;
      bundle.rays.push_back(LidarRay{
        angle,
        hit ? measured_range : max_range,
        max_range,
        hit});
    }
  }

  return bundle;
}

LaserScanPoints ScanBagConverter::convertLaserScan(const sensor_msgs::msg::LaserScan & scan) const
{
  LaserScanPoints points;
  points.endpoints.reserve(scan.ranges.size());
  points.ranges.reserve(scan.ranges.size());
  points.variances.reserve(scan.ranges.size());

  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = static_cast<double>(scan.ranges[i]);
    if (!std::isfinite(range) ||
      range < static_cast<double>(scan.range_min) ||
      range > static_cast<double>(scan.range_max))
    {
      continue;
    }

    const double angle = static_cast<double>(scan.angle_min) +
      static_cast<double>(i) * static_cast<double>(scan.angle_increment);
    points.endpoints.emplace_back(range * std::cos(angle), range * std::sin(angle));
    points.ranges.push_back(range);
    points.variances.push_back(
      options_.sigma_r * options_.sigma_r + range * range * options_.sigma_theta * options_.sigma_theta);
  }

  return points;
}

}  // namespace map_solver
