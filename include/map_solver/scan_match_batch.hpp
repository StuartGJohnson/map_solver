#ifndef MAP_SOLVER__SCAN_MATCH_BATCH_HPP_
#define MAP_SOLVER__SCAN_MATCH_BATCH_HPP_

#include <cstddef>
#include <filesystem>
#include <memory>
#include <vector>

#include "map_solver/scan_bag_converter.hpp"
#include "map_solver/scan_matcher.hpp"

namespace map_solver
{

struct WaypointScan
{
  std::size_t index{0};
  ScanBundle bundle;
};

struct ScanMatchJob
{
  std::size_t target_index{0};
  std::size_t source_index{0};
  std::shared_ptr<const ScanBundle> target;
  std::shared_ptr<const ScanBundle> source;
};

struct ScanMatchJobResult
{
  std::size_t target_index{0};
  std::size_t source_index{0};
  std::filesystem::path target_bag_path;
  std::filesystem::path source_bag_path;
  ScanMatchResult match;
};

std::vector<WaypointScan> loadWaypointScans(
  const std::filesystem::path & mdc_directory,
  const ScanBagConversionOptions & conversion_options);

std::vector<ScanMatchJob> makeSequentialScanMatchJobs(
  const std::vector<WaypointScan> & waypoint_scans);

std::vector<ScanMatchJobResult> runScanMatchJobsParallel(
  const std::vector<ScanMatchJob> & jobs,
  const ScanMatcherOptions & matcher_options,
  std::size_t thread_count = 0);

}  // namespace map_solver

#endif  // MAP_SOLVER__SCAN_MATCH_BATCH_HPP_
