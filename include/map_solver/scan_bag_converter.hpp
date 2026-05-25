#ifndef MAP_SOLVER__SCAN_BAG_CONVERTER_HPP_
#define MAP_SOLVER__SCAN_BAG_CONVERTER_HPP_

#include <filesystem>
#include <string>

#include <sensor_msgs/msg/laser_scan.hpp>

#include "map_solver/localization_search.hpp"

namespace map_solver
{

struct ScanBagConversionOptions
{
  std::string topic{"/scan"};
  std::string storage_id{"mcap"};
  double sigma_r{0.03};
  double sigma_theta{0.005};
  int scan_stride{1};
};

struct ScanBundle
{
  LaserScanPoints points;
  std::size_t scan_count{0};
  std::size_t finite_range_count{0};
};

class ScanBagConverter
{
public:
  explicit ScanBagConverter(ScanBagConversionOptions options = {});

  ScanBundle readWaypointDirectory(const std::filesystem::path & waypoint_directory) const;
  LaserScanPoints convertLaserScan(const sensor_msgs::msg::LaserScan & scan) const;

private:
  ScanBagConversionOptions options_;
};

}  // namespace map_solver

#endif  // MAP_SOLVER__SCAN_BAG_CONVERTER_HPP_
