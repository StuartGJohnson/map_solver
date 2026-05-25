#include "map_solver/sim.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace map_solver
{

namespace
{

void setCell(GeneratedMap & map, int x, int y, std::int8_t value)
{
  if (x < 0 || y < 0 ||
    x >= static_cast<int>(map.msg.info.width) ||
    y >= static_cast<int>(map.msg.info.height))
  {
    return;
  }
  map.msg.data[static_cast<std::size_t>(y) * map.msg.info.width + x] = value;
}

void setOccupiedRectCells(GeneratedMap & map, int min_x, int min_y, int max_x, int max_y)
{
  for (int y = min_y; y <= max_y; ++y) {
    for (int x = min_x; x <= max_x; ++x) {
      setCell(map, x, y, 100);
    }
  }
}

void setOccupiedRectWorld(
  GeneratedMap & map,
  double min_x,
  double min_y,
  double max_x,
  double max_y)
{
  const double resolution = map.msg.info.resolution;
  setOccupiedRectCells(
    map,
    static_cast<int>(std::floor(min_x / resolution)),
    static_cast<int>(std::floor(min_y / resolution)),
    static_cast<int>(std::floor(max_x / resolution)),
    static_cast<int>(std::floor(max_y / resolution)));
}

void addMapBorder(GeneratedMap & map)
{
  const int width = static_cast<int>(map.msg.info.width);
  const int height = static_cast<int>(map.msg.info.height);
  setOccupiedRectCells(map, 0, 0, width - 1, 0);
  setOccupiedRectCells(map, 0, height - 1, width - 1, height - 1);
  setOccupiedRectCells(map, 0, 0, 0, height - 1);
  setOccupiedRectCells(map, width - 1, 0, width - 1, height - 1);
}

void addSymmetricRoom(GeneratedMap & map, double origin_x, double origin_y)
{
  setOccupiedRectWorld(map, origin_x, origin_y, origin_x + 4.0, origin_y + 0.1);
  setOccupiedRectWorld(map, origin_x, origin_y + 4.0, origin_x + 4.0, origin_y + 4.1);
  setOccupiedRectWorld(map, origin_x, origin_y, origin_x + 0.1, origin_y + 4.1);
  setOccupiedRectWorld(map, origin_x + 4.0, origin_y, origin_x + 4.1, origin_y + 4.1);
  setOccupiedRectWorld(map, origin_x + 1.0, origin_y + 2.8, origin_x + 1.6, origin_y + 3.4);
  setOccupiedRectWorld(map, origin_x + 2.7, origin_y + 0.9, origin_x + 2.9, origin_y + 2.1);
}

bool isOccupied(const nav_msgs::msg::OccupancyGrid & map, double wx, double wy)
{
  const auto x = static_cast<int>(std::floor(wx / map.info.resolution));
  const auto y = static_cast<int>(std::floor(wy / map.info.resolution));
  if (x < 0 || y < 0 ||
    x >= static_cast<int>(map.info.width) ||
    y >= static_cast<int>(map.info.height))
  {
    return true;
  }
  return map.data[static_cast<std::size_t>(y) * map.info.width + x] >= 50;
}

}  // namespace

GeneratedMap makeEmptyMap(std::uint32_t width, std::uint32_t height, double resolution)
{
  GeneratedMap map;
  map.msg.header.frame_id = "map";
  map.msg.info.resolution = static_cast<float>(resolution);
  map.msg.info.width = width;
  map.msg.info.height = height;
  map.msg.info.origin.orientation.w = 1.0;
  map.msg.data.assign(static_cast<std::size_t>(width) * height, 0);
  return map;
}

GeneratedMap makeAsymmetricMap()
{
  auto map = makeEmptyMap(90, 70, 0.1);
  addMapBorder(map);

  setOccupiedRectWorld(map, 4.8, 0.9, 5.0, 4.6);
  setOccupiedRectWorld(map, 1.1, 4.0, 1.9, 4.8);
  setOccupiedRectWorld(map, 3.1, 1.0, 3.4, 1.9);
  setOccupiedRectWorld(map, 6.3, 4.4, 7.2, 5.3);
  setOccupiedRectWorld(map, 7.6, 1.0, 7.9, 2.6);
  setOccupiedRectWorld(map, 2.2, 5.6, 2.8, 5.9);
  setOccupiedRectWorld(map, 5.8, 2.1, 6.1, 2.4);
  setOccupiedRectWorld(map, 0.9, 1.2, 1.2, 2.0);
  setOccupiedRectWorld(map, 3.9, 5.0, 4.5, 5.2);
  return map;
}

GeneratedMap makePathologicallySymmetricMap()
{
  auto map = makeEmptyMap(130, 60, 0.1);
  addSymmetricRoom(map, 1.0, 1.0);
  addSymmetricRoom(map, 8.0, 1.0);
  return map;
}

SimulatedLidar simulateLidar(
  const nav_msgs::msg::OccupancyGrid & map,
  const Pose2D & robot_pose,
  SearchOptions & options,
  int beam_count,
  double field_of_view,
  double max_range)
{
  const double variance_r = options.sigma_r * options.sigma_r;
  const double variance_theta = options.sigma_theta * options.sigma_theta;

  SimulatedLidar scan;
  const double min_angle = -0.5 * field_of_view;
  const double angle_step = field_of_view / static_cast<double>(beam_count - 1);
  const double ray_step = 0.5 * map.info.resolution;
  scan.scan.angle_min = static_cast<float>(min_angle);
  scan.scan.angle_max = static_cast<float>(min_angle + angle_step * static_cast<double>(beam_count - 1));
  scan.scan.angle_increment = static_cast<float>(angle_step);
  scan.scan.time_increment = 0.0F;
  scan.scan.scan_time = 0.0F;
  scan.scan.range_min = static_cast<float>(ray_step);
  scan.scan.range_max = static_cast<float>(max_range);
  scan.scan.ranges.assign(static_cast<std::size_t>(beam_count), std::numeric_limits<float>::infinity());
  scan.scan.intensities.clear();

  for (int i = 0; i < beam_count; ++i) {
    const double beam_angle_base = min_angle + static_cast<double>(i) * angle_step;
    const double beam_angle_map = robot_pose.yaw + beam_angle_base;
    const double cos_a = std::cos(beam_angle_map);
    const double sin_a = std::sin(beam_angle_map);

    for (double range = ray_step; range <= max_range; range += ray_step) {
      const double wx = robot_pose.x + range * cos_a;
      const double wy = robot_pose.y + range * sin_a;
      if (isOccupied(map, wx, wy)) {
        scan.scan.ranges[static_cast<std::size_t>(i)] = static_cast<float>(range);
        scan.endpoints_base.endpoints.emplace_back(
          range * std::cos(beam_angle_base),
          range * std::sin(beam_angle_base));
        scan.endpoints_base.ranges.emplace_back(range);
        scan.endpoints_base.variances.emplace_back(
          variance_r + range * range * variance_theta);
        break;
      }
    }
  }

  return scan;
}

SimulatedLidarPair simulateNearbyAsymmetricLidarPair(
  SearchOptions & options,
  int beam_count,
  double field_of_view,
  double max_range)
{
  const auto map = makeAsymmetricMap();
  SimulatedLidarPair pair;
  pair.target_pose = Pose2D{2.45, 2.25, 0.65};
  pair.source_pose = Pose2D{2.78, 2.12, 0.82};
  pair.target = simulateLidar(
    map.msg, pair.target_pose, options, beam_count, field_of_view, max_range);
  pair.source = simulateLidar(
    map.msg, pair.source_pose, options, beam_count, field_of_view, max_range);
  return pair;
}

std::vector<Eigen::Vector2d> transformEndpointsToMap(
  const std::vector<Eigen::Vector2d> & endpoints_base,
  const Pose2D & pose)
{
  std::vector<Eigen::Vector2d> endpoints_map;
  endpoints_map.reserve(endpoints_base.size());
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  for (const auto & endpoint : endpoints_base) {
    endpoints_map.emplace_back(
      pose.x + c * endpoint.x() - s * endpoint.y(),
      pose.y + s * endpoint.x() + c * endpoint.y());
  }
  return endpoints_map;
}

}  // namespace map_solver
