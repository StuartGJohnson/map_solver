#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <string>
#include <vector>

#include "map_solver/scan_bag_converter.hpp"
#include "map_solver/scan_matcher.hpp"
#include "map_solver/sim.hpp"

namespace map_solver
{
namespace
{

void runGnuplotScript(const std::filesystem::path & script_path)
{
  const std::string command = "gnuplot '" + script_path.string() + "' >/dev/null 2>&1";
  const int ret = std::system(command.c_str());
  if (ret == -1) {
    std::cout << "system gnuplot command failed for " << script_path << std::endl;
  }
}

std::filesystem::path artifactDirectory()
{
  const char * test_tmpdir = std::getenv("TEST_TMPDIR");
  auto path = test_tmpdir != nullptr && std::string(test_tmpdir).size() > 0 ?
    std::filesystem::path(test_tmpdir) :
    std::filesystem::temp_directory_path() / "map_solver_tests";
  std::filesystem::create_directories(path);
  return path;
}

void writeRobotFramePlot(const LaserScanPoints & points, const std::string & stem)
{
  const auto dir = artifactDirectory();
  const auto data_path = dir / (stem + ".dat");
  const auto script_path = dir / (stem + ".gp");
  const auto png_path = dir / (stem + ".png");

  {
    std::ofstream data(data_path);
    for (const auto & point : points.endpoints) {
      data << point.x() << ' ' << point.y() << '\n';
    }
  }

  {
    std::ofstream script(script_path);
    script << "set terminal pngcairo size 900,900\n";
    script << "set output '" << png_path.string() << "'\n";
    script << "set size ratio -1\n";
    script << "set title 'scan bundle in robot coordinates'\n";
    script << "plot '" << data_path.string()
           << "' using 1:2 with points pt 7 ps 0.25 lc rgb '#1f77b4' notitle\n";
  }

  runGnuplotScript(script_path);
}

void writeScanMatchCostMap(
  const LaserScanPoints & source,
  const std::optional<Pose2D> & truth,
  const ScanMatchResult & result,
  const std::string & stem)
{
  const auto & cost_map = result.coarse_cost_map;
  const int x_count = static_cast<int>(cost_map.min_score_by_xy.rows());
  const int y_count = static_cast<int>(cost_map.min_score_by_xy.cols());
  if (x_count == 0 || y_count == 0 || source.endpoints.empty()) {
    return;
  }

  const auto dir = artifactDirectory();
  const auto cost_path = dir / (stem + "_cost.dat");
  const auto truth_pose_path = dir / (stem + "_truth_pose.dat");
  const auto est_pose_path = dir / (stem + "_est_pose.dat");
  const auto script_path = dir / (stem + ".gp");
  const auto png_path = dir / (stem + ".png");

  {
    std::ofstream cost(cost_path);
    cost << std::setprecision(12);
    for (int iy = 0; iy < y_count; ++iy) {
      const double y = cost_map.min_y + static_cast<double>(iy) * cost_map.xy_step;
      for (int ix = 0; ix < x_count; ++ix) {
        const double x = cost_map.min_x + static_cast<double>(ix) * cost_map.xy_step;
        cost << x << ' ' << y << ' '
             << cost_map.min_score_by_xy(ix, iy) / static_cast<double>(source.endpoints.size())
             << '\n';
      }
      cost << '\n';
    }
  }

  {
    std::ofstream truth_pose(truth_pose_path);
    if (truth.has_value()) {
      truth_pose << truth->x << ' ' << truth->y << ' '
                 << 0.16 * std::cos(truth->yaw) << ' ' << 0.16 * std::sin(truth->yaw)
                 << '\n';
    }

    std::ofstream est_pose(est_pose_path);
    if (result.success) {
      est_pose << result.target_from_source.x << ' ' << result.target_from_source.y << ' '
               << 0.12 * std::cos(result.target_from_source.yaw) << ' '
               << 0.12 * std::sin(result.target_from_source.yaw) << '\n';
    }
  }

  {
    std::ofstream script(script_path);
    script << "set terminal pngcairo size 1000,800\n";
    script << "set output '" << png_path.string() << "'\n";
    script << "set size ratio -1\n";
    script << "set title 'scan-match cost: min score over yaw'\n";
    script << "set xlabel 'relative x (m)'\n";
    script << "set ylabel 'relative y (m)'\n";
    script << "set cblabel 'min negative log likelihood per point'\n";
    script << "set palette defined (0 '#ffffcc', 0.45 '#41b6c4', 1 '#253494')\n";
    script << "set key outside\n";
    script << "plot '" << cost_path.string()
           << "' using 1:2:3 with image title 'cost'";
    if (truth.has_value()) {
      script << ", \\\n";
      script << "     '" << truth_pose_path.string()
             << "' using 1:2:3:4 with vectors head filled lw 3 lc rgb '#2ca02c' title 'truth'";
    }
    script << ", \\\n";
    script << "     '" << est_pose_path.string()
           << "' using 1:2:3:4 with vectors head filled lw 3 lc rgb '#d62728' title 'est'\n";
  }

  runGnuplotScript(script_path);
}

Eigen::Vector2d transformPoint(const Eigen::Vector2d & point, const Pose2D & pose)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return Eigen::Vector2d{
    pose.x + c * point.x() - s * point.y(),
    pose.y + s * point.x() + c * point.y()};
}

void writeAlignedScanPlot(
  const LaserScanPoints & target,
  const LaserScanPoints & source,
  const ScanMatchResult & result,
  const std::string & stem)
{
  const auto dir = artifactDirectory();
  const auto target_path = dir / (stem + "_waypoint_0000.dat");
  const auto source_path = dir / (stem + "_waypoint_0001_transformed.dat");
  const auto script_path = dir / (stem + ".gp");
  const auto png_path = dir / (stem + ".png");

  {
    std::ofstream target_data(target_path);
    for (const auto & point : target.endpoints) {
      target_data << point.x() << ' ' << point.y() << '\n';
    }
  }

  {
    std::ofstream source_data(source_path);
    for (const auto & point : source.endpoints) {
      const auto transformed = transformPoint(point, result.target_from_source);
      source_data << transformed.x() << ' ' << transformed.y() << '\n';
    }
  }

  {
    std::ofstream script(script_path);
    script << "set terminal pngcairo size 1000,900\n";
    script << "set output '" << png_path.string() << "'\n";
    script << "set size ratio -1\n";
    script << "set title 'real scan alignment in waypoint 0000 frame'\n";
    script << "set xlabel 'x (m)'\n";
    script << "set ylabel 'y (m)'\n";
    script << "set key outside\n";
    script << "plot '" << target_path.string()
           << "' using 1:2 with points pt 7 ps 0.18 lc rgb '#2ca02c' title 'waypoint 0000', \\\n";
    script << "     '" << source_path.string()
           << "' using 1:2 with points pt 7 ps 0.18 lc rgb '#d62728' title 'waypoint 0001 transformed'\n";
  }

  runGnuplotScript(script_path);
}

std::filesystem::path firstMdcDataDirectory()
{
  const auto data_dir = std::filesystem::path(MAP_SOLVER_SOURCE_DIR) / "data";
  if (!std::filesystem::exists(data_dir)) {
    return {};
  }

  std::vector<std::filesystem::path> candidates;
  for (const auto & entry : std::filesystem::directory_iterator(data_dir)) {
    if (entry.is_directory() && entry.path().filename().string().rfind("mdc", 0) == 0) {
      candidates.push_back(entry.path());
    }
  }
  std::sort(candidates.begin(), candidates.end());
  return candidates.empty() ? std::filesystem::path{} : candidates.front();
}

TEST(SimPipelineTest, GeneratesNearbyAsymmetricScanPair)
{
  SearchOptions options;
  options.sigma_r = 0.03;
  options.sigma_theta = 0.005;
  const auto pair = simulateNearbyAsymmetricLidarPair(options);

  EXPECT_GE(pair.target.endpoints_base.endpoints.size(), 120U);
  EXPECT_GE(pair.source.endpoints_base.endpoints.size(), 120U);

  const auto truth = relativePose(pair.target_pose, pair.source_pose);
  EXPECT_NEAR(truth.x, 0.1840, 0.01);
  EXPECT_NEAR(truth.y, -0.3067, 0.01);
  EXPECT_NEAR(truth.yaw, 0.17, 1.0e-9);
}

TEST(ScanMatcherFunctionalTest, RecoversKnownRelativePose)
{
  SearchOptions sim_options;
  sim_options.sigma_r = 0.03;
  sim_options.sigma_theta = 0.005;
  const auto pair = simulateNearbyAsymmetricLidarPair(sim_options, 121, kTwoPi, 5.5);
  const auto truth = relativePose(pair.target_pose, pair.source_pose);

  ScanMatcherOptions options;
  options.k_neighbors = 5;
  options.max_translation = 0.8;
  options.coarse_xy_step = 0.15;
  options.coarse_yaw_step = 5.0 * kPi / 180.0;
  options.refine_levels = 5;
  options.max_accepted_cost_per_point = 100.0;

  const ScanMatcher matcher(options);
  const auto result = matcher.match(pair.target.endpoints_base, pair.source.endpoints_base);
  writeScanMatchCostMap(
    pair.source.endpoints_base,
    std::optional<Pose2D>{truth},
    result,
    "asymmetric_scan_match_cost");

  auto translation_error = std::hypot(result.target_from_source.x - truth.x, result.target_from_source.y - truth.y);
  auto yaw_error = yawError(result.target_from_source.yaw, truth.yaw);

  std::cout << "translation error = " << translation_error << std::endl;
  std::cout << "yaw error = " << yaw_error << std::endl;

  ASSERT_TRUE(result.success) << result.message
                              << " cost_per_point=" << result.diagnostics.cost_per_point;
  EXPECT_LT(translation_error, 0.12);
  EXPECT_LT(yaw_error, 3.0 * kPi / 180.0);
  EXPECT_TRUE(result.covariance.allFinite());

  const auto edge = matcher.makePoseGraphEdge(0, 1, result);
  EXPECT_EQ(edge.target_key, 0U);
  EXPECT_EQ(edge.source_key, 1U);
}

TEST(ScanBagConverterTest, ConvertsWaypointBagScanTopic)
{
  const auto source_dir = std::filesystem::path(MAP_SOLVER_SOURCE_DIR);
  const auto waypoint_dir = source_dir / "data" / "mdc_20260520_135327" / "waypoint_0000";
  if (!std::filesystem::exists(waypoint_dir)) {
    GTEST_SKIP() << "waypoint bag data not present";
  }

  ScanBagConversionOptions options;
  options.scan_stride = 10;
  const ScanBagConverter converter(options);
  const auto bundle = converter.readWaypointDirectory(waypoint_dir);
  writeRobotFramePlot(bundle.points, "waypoint_0000_scan_robot");

  EXPECT_GT(bundle.scan_count, 0U);
  EXPECT_GT(bundle.points.endpoints.size(), 50U);
  EXPECT_EQ(bundle.points.endpoints.size(), bundle.points.ranges.size());
  EXPECT_EQ(bundle.points.endpoints.size(), bundle.points.variances.size());
}

TEST(RealDataScanMatcherTest, AlignsWaypoint0000And0001)
{
  const auto mdc_dir = firstMdcDataDirectory();
  if (mdc_dir.empty()) {
    GTEST_SKIP() << "mdc data directory not present";
  }

  const auto waypoint_0000 = mdc_dir / "waypoint_0000";
  const auto waypoint_0001 = mdc_dir / "waypoint_0001";
  if (!std::filesystem::exists(waypoint_0000) || !std::filesystem::exists(waypoint_0001)) {
    GTEST_SKIP() << "waypoint_0000/waypoint_0001 data not present";
  }

  ScanBagConversionOptions conversion_options;
  conversion_options.scan_stride = 3;
  conversion_options.sigma_r = 0.03;
  conversion_options.sigma_theta = 0.005;
  const ScanBagConverter converter(conversion_options);
  const auto bundle_0000 = converter.readWaypointDirectory(waypoint_0000);
  const auto bundle_0001 = converter.readWaypointDirectory(waypoint_0001);

  ASSERT_GT(bundle_0000.scan_count, 0U);
  ASSERT_GT(bundle_0001.scan_count, 0U);
  ASSERT_GT(bundle_0000.points.endpoints.size(), 100U);
  ASSERT_GT(bundle_0001.points.endpoints.size(), 100U);

  ScanMatcherOptions match_options;
  match_options.k_neighbors = 5;
  match_options.max_translation = 1.5;
  match_options.coarse_xy_step = 0.25;
  match_options.coarse_yaw_step = 5.0 * kPi / 180.0;
  match_options.refine_levels = 5;
  match_options.max_accepted_cost_per_point = 200.0;

  const ScanMatcher matcher(match_options);
  const auto result = matcher.match(bundle_0000.points, bundle_0001.points);
  writeScanMatchCostMap(
    bundle_0001.points,
    std::nullopt,
    result,
    "real_waypoint_0000_0001_scan_match_cost");
  writeAlignedScanPlot(
    bundle_0000.points,
    bundle_0001.points,
    result,
    "real_waypoint_0000_0001_aligned_scans");

  std::cout << "real data estimate: x=" << result.target_from_source.x
            << " y=" << result.target_from_source.y
            << " yaw=" << result.target_from_source.yaw
            << " cost_per_point=" << result.diagnostics.cost_per_point
            << " evaluated_pose_count=" << result.diagnostics.evaluated_pose_count
            << std::endl;

  ASSERT_TRUE(result.success) << result.message
                              << " cost_per_point=" << result.diagnostics.cost_per_point;
  EXPECT_TRUE(result.covariance.allFinite());

  std::cout << "covariance: " << result.covariance << std::endl;
}

}  // namespace
}  // namespace map_solver
