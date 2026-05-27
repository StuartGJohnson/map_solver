#ifndef MAP_SOLVER__OCCUPANCY_MAP_BUILDER_HPP_
#define MAP_SOLVER__OCCUPANCY_MAP_BUILDER_HPP_

#include <cstdint>
#include <vector>

#include "map_solver/pose_graph.hpp"
#include "map_solver/scan_match_batch.hpp"

namespace map_solver
{

struct OccupancyMapOptions
{
  double resolution{0.005};
  double map_padding_meters{1.0};
  double p_free{0.35};
  double p_occ{0.7};
  double max_no_return_range{0.0};
  double min_probability{0.001};
  double max_probability{0.999};
};

struct OccupancyMap
{
  double resolution{0.005};
  double origin_x{0.0};
  double origin_y{0.0};
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::vector<double> occupancy_probability;
};

OccupancyMap buildOccupancyMap(
  const std::vector<WaypointScan> & waypoint_scans,
  const std::vector<OptimizedPose2D> & poses,
  const OccupancyMapOptions & options = {});

}  // namespace map_solver

#endif  // MAP_SOLVER__OCCUPANCY_MAP_BUILDER_HPP_
