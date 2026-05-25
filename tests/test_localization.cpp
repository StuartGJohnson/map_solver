#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "map_solver/localization_search.hpp"
#include "map_solver/map_model.hpp"
#include "map_solver/sim.hpp"

namespace map_solver
{
namespace
{

MapModel buildMapModel(const nav_msgs::msg::OccupancyGrid & map)
{
  MapModel model;
  MapBuildOptions options;
  options.occupied_threshold = 50;
  options.unknown_is_occupied = false;
  EXPECT_TRUE(model.update(map, options));
  return model;
}

SearchOptions makeFunctionalTestSearchOptions()
{
  SearchOptions options;
  options.coarse_xy_step = 0.1;
  options.coarse_yaw_step = 5.0 * kPi / 180.0;
  options.top_k = 40;
  options.candidate_min_xy_separation = 0.45;
  options.candidate_min_yaw_separation = 8.0 * kPi / 180.0;
  options.refine_levels = 4;
  options.refine_xy_step = 0.15;
  options.refine_yaw_step = 4.0 * kPi / 180.0;
  options.off_map_distance = 3.0;
  options.free_space_weight = 0.4;
  options.free_space_sample_step = 0.2;
  options.min_endpoint_count = 80;
  options.covariance_regularization = 1.0e-2;
  options.sigma_r = 0.03;
  options.sigma_theta = 0.005;
  options.alpha_m = 1.0;
  return options;
}

std::filesystem::path testArtifactDirectory()
{
  const char * test_tmpdir = std::getenv("TEST_TMPDIR");
  if (test_tmpdir != nullptr && std::string(test_tmpdir).size() > 0) {
    std::filesystem::create_directories(test_tmpdir);
    return test_tmpdir;
  }

  auto path = std::filesystem::temp_directory_path() / "global_robot_localization_tests";
  std::filesystem::create_directories(path);
  return path;
}

void writeGnuplotArtifacts(
  const std::string & name,
  const std::string & title,
  const nav_msgs::msg::OccupancyGrid & map,
  const MapModel & model,
  LocalizationSearch & search,
  const Pose2D & truth,
  const std::vector<Eigen::Vector2d> & endpoints_base,
  const SearchResult & result)
{
  const auto dir = testArtifactDirectory();
  const auto cost_path = dir / (name + "_yaw_min_cost.dat");
  const auto occupied_path = dir / (name + "_occupied.dat");
  const auto poses_path_truth = dir / (name + "_poses_truth.dat");
  const auto poses_path_est = dir / (name + "_poses_est.dat");
  const auto endpoints_path = dir / (name + "_lidar_endpoints_map.dat");
  const auto script_path = dir / (name + ".gp");
  const auto png_path = dir / (name + ".png");

  {
    search.dumpCoarseScore(cost_path);
  }

  {
    std::ofstream occupied(occupied_path);
    for (std::uint32_t y = 0; y < map.info.height; ++y) {
      for (std::uint32_t x = 0; x < map.info.width; ++x) {
        if (map.data[static_cast<std::size_t>(y) * map.info.width + x] < 50) {
          continue;
        }
        occupied << (static_cast<double>(x) + 0.5) * map.info.resolution << ' '
                 << (static_cast<double>(y) + 0.5) * map.info.resolution << '\n';
      }
    }
  }

  {
    std::ofstream poses_truth(poses_path_truth);
    poses_truth << truth.x << ' ' << truth.y << ' '
           << 0.5 * std::cos(truth.yaw) << ' ' << 0.5 * std::sin(truth.yaw)
           << " truth\n";
    std::ofstream poses_est(poses_path_est);
    if (result.success) {
      poses_est << result.best.pose.x << ' ' << result.best.pose.y << ' '
            << 0.35 * std::cos(result.best.pose.yaw) << ' '
            << 0.35 * std::sin(result.best.pose.yaw) << " estimated\n";
    }
  }

  {
    std::ofstream endpoints(endpoints_path);
    const auto endpoints_map = transformEndpointsToMap(endpoints_base, truth);
    for (const auto & endpoint : endpoints_map) {
      endpoints << endpoint.x() << ' ' << endpoint.y() << '\n';
    }
  }

  {
    //double step = search.xy_step_cells * search.resolution;
    std::ofstream script(script_path);
    script << "set terminal pngcairo size 1000,700\n";
    script << "set output '" << png_path.string() << "'\n";
    script << "set size ratio -1\n";
    script << "set title '" << title << "'\n";
    script << "set key outside\n";
    script << "set palette defined (0 '#f7fcf0', 0.5 '#addd8e', 1 '#2b8cbe')\n";
    script << "set cbrange [20:1000]\n";
    script << "set cblabel 'min J over yaw'\n";
    script << "set style fill transparent solid 0.4\n"; // 40% opaque
 
    // Start the plot command
    script << "plot '" << cost_path.string() << "' with image title 'cost', \\\n";
    script << "     '" << occupied_path.string() << "' using 1:2 with points pt 5 ps 0.35 lc rgb '#303030' title 'occupied', \\\n";
    script << "     '" << poses_path_truth.string() << "' every ::0::0 using 1:2:3:4 with vectors head filled lw 3 lc rgb '#1f77b4' title 'truth', \\\n";
    script << "     '" << poses_path_est.string() << "' every ::0::0 using 1:2:3:4 with vectors head filled lw 3 lc rgb '#2ca02c' title 'estimated', \\\n";
    script << "     '" << endpoints_path.string() << "' using 1:2 with points pt 7 ps 0.45 lc rgb '#d62728' title 'lidar endpoints'\n";
  }

  const std::string command = "gnuplot '" + script_path.string() + "' >/dev/null 2>&1";
  // todo: deal with issues here
  int ret = std::system(command.c_str());
  if (ret == -1) {
    std::cout << "system gnuplot command failed!" << std::endl;
  }
}

bool containsPoseNear(
  const std::vector<Candidate> & candidates,
  const Pose2D & pose,
  double xy_tolerance,
  double yaw_tolerance)
{
  return std::any_of(candidates.begin(), candidates.end(), [&](const Candidate & candidate) {
    const double dx = candidate.pose.x - pose.x;
    const double dy = candidate.pose.y - pose.y;
    return std::hypot(dx, dy) <= xy_tolerance &&
           yawError(candidate.pose.yaw, pose.yaw) <= yaw_tolerance;
  });
}

TEST(LocalizationFunctionalTest, LocalizesAsymmetricMap)
{
  // this test places graphical output in:
  // /tmp/global_robot_localization_tests/asymmetric_localization.png
  // it is useful to watch this file in your ide (or otherwise)
  auto options = makeFunctionalTestSearchOptions();

  const auto generated = makeAsymmetricMap();
  const auto model = buildMapModel(generated.msg);
  const Pose2D truth{2.45, 2.25, 0.65};
  const auto scan = simulateLidar(generated.msg, truth, options, 91, 2.0 * kPi, 5.5);
  ASSERT_GE(scan.endpoints_base.endpoints.size(), 70U);

  LocalizationSearch search(options);
  const auto result = search.run(model, scan.endpoints_base);
  const double truth_cost = search.scorePose(model, scan.endpoints_base, truth);
  writeGnuplotArtifacts(
    "asymmetric_localization",
    "asymmetric localization",
    generated.msg, model, search, truth,
    scan.endpoints_base.endpoints, result);

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_LT(std::hypot(result.best.pose.x - truth.x, result.best.pose.y - truth.y), 0.18);
  std::cout << "estimated pose: x=" << result.best.pose.x << " y=" << result.best.pose.y
    << " yaw=" << result.best.pose.yaw << " cost=" << result.best.cost
    << " truth_cost=" << truth_cost << std::endl;
  EXPECT_LT(yawError(result.best.pose.yaw, truth.yaw), 5.0 * kPi / 180.0);
  std::cout << "covariance: " << result.covariance  << std::endl;

  EXPECT_TRUE(result.covariance.allFinite());
  EXPECT_GT(result.covariance(0, 0), 0.0);
  EXPECT_GT(result.covariance(1, 1), 0.0);
  EXPECT_GT(result.covariance(2, 2), 0.0);
}

TEST(LocalizationFunctionalTest, ExposesMultipleGlobalMinimaInSymmetricMap)
{
  // this test places graphical output in:
  // /tmp/global_robot_localization_tests/symmetric_localization.png
  // it is useful to watch this file in your ide (or otherwise)
  auto options = makeFunctionalTestSearchOptions();
  const auto generated = makePathologicallySymmetricMap();
  const auto model = buildMapModel(generated.msg);
  const Pose2D truth{2.8, 2.6, 0.35};
  const Pose2D equivalent{9.8, 2.6, 0.35};
  const auto scan = simulateLidar(generated.msg, truth, options, 91, 2.0 * kPi, 4.0);
  ASSERT_GE(scan.endpoints_base.endpoints.size(), 70U);

  options.top_k = 24;
  LocalizationSearch search(options);
  const auto result = search.run(model, scan.endpoints_base);
  const double truth_cost = search.scorePose(model, scan.endpoints_base, truth);
  writeGnuplotArtifacts(
    "symmetric_localization",
    "symmetric localization",
    generated.msg, model, search, truth,
    scan.endpoints_base.endpoints, result);

  std::cout << "estimated pose: x=" << result.best.pose.x << " y=" << result.best.pose.y
    << " yaw=" << result.best.pose.yaw << " cost=" << result.best.cost
    << " truth_cost=" << truth_cost << std::endl;    
  std::cout << "covariance: " << result.covariance  << std::endl;  

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_TRUE(containsPoseNear(result.candidates, truth, 0.25, 6.0 * kPi / 180.0));
  EXPECT_TRUE(containsPoseNear(result.candidates, equivalent, 0.25, 6.0 * kPi / 180.0));
  ASSERT_GE(result.candidates.size(), 2U);
  EXPECT_LT(std::abs(result.candidates.front().cost - result.candidates[1].cost), 0.02);
}

}  // namespace
}  // namespace map_solver
