#include "map_solver/occupancy_map_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace map_solver
{
namespace
{

double clampProbability(double p, const OccupancyMapOptions & options)
{
  return std::clamp(p, options.min_probability, options.max_probability);
}

double logit(double p)
{
  return std::log(p / (1.0 - p));
}

double probabilityFromLogOdds(double log_odds)
{
  if (log_odds >= 0.0) {
    const double e = std::exp(-log_odds);
    return 1.0 / (1.0 + e);
  }
  const double e = std::exp(log_odds);
  return e / (1.0 + e);
}

Pose2D poseForIndex(const std::vector<OptimizedPose2D> & poses, std::size_t index)
{
  const auto it = std::find_if(
    poses.begin(), poses.end(),
    [&](const OptimizedPose2D & pose) {
      return pose.index == index;
    });
  if (it == poses.end()) {
    throw std::runtime_error("missing optimized pose for waypoint index");
  }
  return it->pose;
}

void rayEndpoint(
  const Pose2D & pose,
  const LidarRay & ray,
  const OccupancyMapOptions & options,
  double & end_x,
  double & end_y)
{
  double range = ray.range;
  if (!ray.hit && options.max_no_return_range > 0.0) {
    range = std::min(range, options.max_no_return_range);
  }

  const double angle = pose.yaw + ray.angle;
  end_x = pose.x + range * std::cos(angle);
  end_y = pose.y + range * std::sin(angle);
}

int cellIndex(double value, double origin, double resolution)
{
  return static_cast<int>(std::floor((value - origin) / resolution));
}

void addCellLogOdds(
  std::vector<double> & log_odds,
  std::uint32_t width,
  std::uint32_t height,
  int x,
  int y,
  double delta)
{
  if (x < 0 || y < 0 || x >= static_cast<int>(width) || y >= static_cast<int>(height)) {
    return;
  }
  log_odds[static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x)] += delta;
}

void traceFreeCells(
  std::vector<double> & log_odds,
  std::uint32_t width,
  std::uint32_t height,
  int x0,
  int y0,
  int x1,
  int y1,
  bool mark_endpoint,
  double free_delta,
  double occupied_delta)
{
  int x = x0;
  int y = y0;
  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;

  while (true) {
    if (x == x1 && y == y1) {
      if (mark_endpoint) {
        addCellLogOdds(log_odds, width, height, x, y, occupied_delta);
      } else {
        addCellLogOdds(log_odds, width, height, x, y, free_delta);
      }
      break;
    }

    addCellLogOdds(log_odds, width, height, x, y, free_delta);
    const int twice_error = 2 * error;
    if (twice_error >= dy) {
      error += dy;
      x += sx;
    }
    if (twice_error <= dx) {
      error += dx;
      y += sy;
    }
  }
}

}  // namespace

OccupancyMap buildOccupancyMap(
  const std::vector<WaypointScan> & waypoint_scans,
  const std::vector<OptimizedPose2D> & poses,
  const OccupancyMapOptions & options)
{
  if (waypoint_scans.empty() || poses.empty()) {
    return {};
  }
  if (options.resolution <= 0.0) {
    throw std::runtime_error("occupancy map resolution must be positive");
  }

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();

  for (const auto & waypoint_scan : waypoint_scans) {
    const Pose2D pose = poseForIndex(poses, waypoint_scan.index);
    min_x = std::min(min_x, pose.x);
    min_y = std::min(min_y, pose.y);
    max_x = std::max(max_x, pose.x);
    max_y = std::max(max_y, pose.y);
    for (const auto & ray : waypoint_scan.bundle.rays) {
      double end_x = 0.0;
      double end_y = 0.0;
      rayEndpoint(pose, ray, options, end_x, end_y);
      min_x = std::min(min_x, end_x);
      min_y = std::min(min_y, end_y);
      max_x = std::max(max_x, end_x);
      max_y = std::max(max_y, end_y);
    }
  }

  OccupancyMap map;
  map.resolution = options.resolution;
  map.origin_x = min_x - options.map_padding_meters;
  map.origin_y = min_y - options.map_padding_meters;
  map.width = static_cast<std::uint32_t>(
    std::ceil((max_x - min_x + 2.0 * options.map_padding_meters) / options.resolution)) + 1U;
  map.height = static_cast<std::uint32_t>(
    std::ceil((max_y - min_y + 2.0 * options.map_padding_meters) / options.resolution)) + 1U;

  std::vector<double> log_odds(static_cast<std::size_t>(map.width) * map.height, 0.0);
  const double free_delta = logit(clampProbability(options.p_free, options));
  const double occupied_delta = logit(clampProbability(options.p_occ, options));

  for (const auto & waypoint_scan : waypoint_scans) {
    const Pose2D pose = poseForIndex(poses, waypoint_scan.index);
    const int start_x = cellIndex(pose.x, map.origin_x, map.resolution);
    const int start_y = cellIndex(pose.y, map.origin_y, map.resolution);
    for (const auto & ray : waypoint_scan.bundle.rays) {
      double end_x = 0.0;
      double end_y = 0.0;
      rayEndpoint(pose, ray, options, end_x, end_y);
      const int end_cell_x = cellIndex(end_x, map.origin_x, map.resolution);
      const int end_cell_y = cellIndex(end_y, map.origin_y, map.resolution);
      traceFreeCells(
        log_odds,
        map.width,
        map.height,
        start_x,
        start_y,
        end_cell_x,
        end_cell_y,
        ray.hit,
        free_delta,
        occupied_delta);
    }
  }

  map.occupancy_probability.resize(log_odds.size());
  std::transform(
    log_odds.begin(), log_odds.end(), map.occupancy_probability.begin(),
    [](double value) {
      return probabilityFromLogOdds(value);
    });
  return map;
}

}  // namespace map_solver
