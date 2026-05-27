#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Eigenvalues>

#include "map_solver/occupancy_map_builder.hpp"
#include "map_solver/pose_graph.hpp"
#include "map_solver/scan_bag_converter.hpp"
#include "map_solver/scan_match_batch.hpp"
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
  const std::string & title,
  const std::string & target_label,
  const std::string & source_label,
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
    script << "set title '" << title << "'\n";
    script << "set xlabel 'x (m)'\n";
    script << "set ylabel 'y (m)'\n";
    script << "set key outside\n";
    script << "plot '" << target_path.string()
           << "' using 1:2 with points pt 7 ps 0.18 lc rgb '#2ca02c' title '"
           << target_label << "', \\\n";
    script << "     '" << source_path.string()
           << "' using 1:2 with points pt 7 ps 0.18 lc rgb '#d62728' title '"
           << source_label << "'\n";
  }

  runGnuplotScript(script_path);
}

void writeOptimizedPosePlot(
  const std::vector<OptimizedPose2D> & poses,
  const std::string & stem)
{
  const auto dir = artifactDirectory();
  const auto pose_path = dir / (stem + "_poses.dat");
  const auto label_path = dir / (stem + "_labels.dat");
  const auto covariance_path = dir / (stem + "_covariance_ellipses.dat");
  const auto script_path = dir / (stem + ".gp");
  const auto png_path = dir / (stem + ".png");

  {
    std::ofstream pose_data(pose_path);
    for (const auto & pose : poses) {
      pose_data << pose.pose.x << ' ' << pose.pose.y << ' '
                << 0.18 * std::cos(pose.pose.yaw) << ' '
                << 0.18 * std::sin(pose.pose.yaw) << '\n';
    }
  }

  {
    std::ofstream label_data(label_path);
    for (const auto & pose : poses) {
      label_data << pose.pose.x << ' ' << pose.pose.y << " \""
                 << pose.index << "\"\n";
    }
  }

  {
    std::ofstream covariance_data(covariance_path);
    constexpr double kSigmaScale = 2.0;
    constexpr int kEllipseSamples = 72;
    for (const auto & pose : poses) {
      const Eigen::Matrix2d xy_covariance =
        pose.covariance.topLeftCorner<2, 2>();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(xy_covariance);
      if (solver.info() != Eigen::Success) {
        continue;
      }

      const Eigen::Vector2d radii{
        kSigmaScale * std::sqrt(std::max(0.0, solver.eigenvalues()(0))),
        kSigmaScale * std::sqrt(std::max(0.0, solver.eigenvalues()(1)))};
      const Eigen::Matrix2d axes = solver.eigenvectors();
      for (int i = 0; i <= kEllipseSamples; ++i) {
        const double angle = kTwoPi * static_cast<double>(i) /
          static_cast<double>(kEllipseSamples);
        const Eigen::Vector2d unit{std::cos(angle), std::sin(angle)};
        const Eigen::Vector2d offset = axes * radii.asDiagonal() * unit;
        covariance_data << pose.pose.x + offset.x() << ' '
                        << pose.pose.y + offset.y() << '\n';
      }
      covariance_data << '\n';
    }
  }

  {
    std::ofstream script(script_path);
    script << "set terminal pngcairo size 1000,800\n";
    script << "set output '" << png_path.string() << "'\n";
    script << "set size ratio -1\n";
    script << "set title 'optimized waypoint poses'\n";
    script << "set xlabel 'x (m)'\n";
    script << "set ylabel 'y (m)'\n";
    script << "set key outside\n";
    script << "plot '" << pose_path.string()
           << "' using 1:2 with linespoints lw 2 pt 7 ps 0.8 lc rgb '#1f77b4' title 'trajectory', \\\n";
    script << "     '" << covariance_path.string()
           << "' using 1:2 with lines lw 1.5 lc rgb '#2ca02c' title '2 sigma marginal covariance', \\\n";
    script << "     '" << pose_path.string()
           << "' using 1:2:3:4 with vectors head filled lw 2 lc rgb '#d62728' title 'heading', \\\n";
    script << "     '" << label_path.string()
           << "' using 1:2:3 with labels offset char 0.6,0.6 notitle\n";
  }

  runGnuplotScript(script_path);
}

void writeOccupancyMapPlot(
  const OccupancyMap & map,
  const std::vector<OptimizedPose2D> & poses,
  const std::string & title,
  const std::string & stem)
{
  const auto dir = artifactDirectory();
  const auto map_path = dir / (stem + "_occupancy.dat");
  const auto pose_path = dir / (stem + "_poses.dat");
  const auto script_path = dir / (stem + ".gp");
  const auto png_path = dir / (stem + ".png");

  {
    std::ofstream data(map_path);
    data << std::setprecision(8);
    for (std::uint32_t y = 0; y < map.height; ++y) {
      const double world_y = map.origin_y + (static_cast<double>(y) + 0.5) * map.resolution;
      for (std::uint32_t x = 0; x < map.width; ++x) {
        const double world_x = map.origin_x + (static_cast<double>(x) + 0.5) * map.resolution;
        const double p = map.occupancy_probability[
          static_cast<std::size_t>(y) * map.width + x];
        data << world_x << ' ' << world_y << ' ' << p << '\n';
      }
      data << '\n';
    }
  }

  {
    std::ofstream pose_data(pose_path);
    for (const auto & pose : poses) {
      pose_data << pose.pose.x << ' ' << pose.pose.y << ' '
                << 0.15 * std::cos(pose.pose.yaw) << ' '
                << 0.15 * std::sin(pose.pose.yaw) << '\n';
    }
  }

  {
    std::ofstream script(script_path);
    script << "set terminal pngcairo size 1200,900\n";
    script << "set output '" << png_path.string() << "'\n";
    script << "set size ratio -1\n";
    script << "set title '" << title << "'\n";
    script << "set xlabel 'x (m)'\n";
    script << "set ylabel 'y (m)'\n";
    script << "set cbrange [0:1]\n";
    script << "set cblabel 'p(occupied)'\n";
    script << "set palette gray negative\n";
    script << "set key outside\n";
    script << "plot '" << map_path.string()
           << "' using 1:2:3 with image title 'occupancy', \\\n";
    script << "     '" << pose_path.string()
           << "' using 1:2 with linespoints lw 2 pt 7 ps 0.5 lc rgb '#1f77b4' title 'trajectory', \\\n";
    script << "     '" << pose_path.string()
           << "' using 1:2:3:4 with vectors head filled lw 2 lc rgb '#d62728' title 'heading'\n";
  }

  runGnuplotScript(script_path);
}

void writeGlobalEndpointPlot(
  const std::vector<WaypointScan> & waypoint_scans,
  const std::vector<OptimizedPose2D> & poses,
  const std::string & title,
  const std::string & stem)
{
  const auto dir = artifactDirectory();
  const auto endpoints_path = dir / (stem + "_endpoints.dat");
  const auto pose_path = dir / (stem + "_poses.dat");
  const auto script_path = dir / (stem + ".gp");
  const auto png_path = dir / (stem + ".png");

  {
    std::ofstream endpoints_data(endpoints_path);
    for (const auto & waypoint_scan : waypoint_scans) {
      const auto pose_it = std::find_if(
        poses.begin(), poses.end(),
        [&](const OptimizedPose2D & pose) {
          return pose.index == waypoint_scan.index;
        });
      if (pose_it == poses.end()) {
        continue;
      }
      for (const auto & point : waypoint_scan.bundle.points.endpoints) {
        const auto transformed = transformPoint(point, pose_it->pose);
        endpoints_data << transformed.x() << ' ' << transformed.y() << ' '
                       << waypoint_scan.index << '\n';
      }
      endpoints_data << '\n';
    }
  }

  {
    std::ofstream pose_data(pose_path);
    for (const auto & pose : poses) {
      pose_data << pose.pose.x << ' ' << pose.pose.y << ' '
                << 0.15 * std::cos(pose.pose.yaw) << ' '
                << 0.15 * std::sin(pose.pose.yaw) << '\n';
    }
  }

  {
    std::ofstream script(script_path);
    script << "set terminal pngcairo size 1200,900\n";
    script << "set output '" << png_path.string() << "'\n";
    script << "set size ratio -1\n";
    script << "set title '" << title << "'\n";
    script << "set xlabel 'x (m)'\n";
    script << "set ylabel 'y (m)'\n";
    script << "set key outside\n";
    script << "plot '" << endpoints_path.string()
           << "' using 1:2:3 with points pt 7 ps 0.14 palette title 'lidar endpoints', \\\n";
    script << "     '" << pose_path.string()
           << "' using 1:2 with linespoints lw 2 pt 7 ps 0.6 lc rgb '#000000' title 'trajectory', \\\n";
    script << "     '" << pose_path.string()
           << "' using 1:2:3:4 with vectors head filled lw 2 lc rgb '#d62728' title 'heading'\n";
  }

  runGnuplotScript(script_path);
}

std::vector<OptimizedPose2D> composeRawSequentialScanMatchPoses(
  const std::vector<ScanMatchJobResult> & edges)
{
  std::vector<OptimizedPose2D> poses;
  poses.push_back(OptimizedPose2D{0U, Pose2D{0.0, 0.0, 0.0}});
  for (const auto & edge : edges) {
    const auto target_pose = std::find_if(
      poses.begin(), poses.end(),
      [&](const OptimizedPose2D & pose) {
        return pose.index == edge.target_index;
      });
    if (target_pose == poses.end()) {
      continue;
    }

    const auto source_pose = std::find_if(
      poses.begin(), poses.end(),
      [&](const OptimizedPose2D & pose) {
        return pose.index == edge.source_index;
      });
    if (source_pose != poses.end()) {
      continue;
    }

    poses.push_back(OptimizedPose2D{
      edge.source_index,
      composePoses(target_pose->pose, edge.match.target_from_source)});
  }

  std::sort(
    poses.begin(), poses.end(),
    [](const OptimizedPose2D & lhs, const OptimizedPose2D & rhs) {
      return lhs.index < rhs.index;
    });
  return poses;
}

std::vector<ScanMatchJob> makeSkipOneScanMatchJobs(
  const std::vector<WaypointScan> & waypoint_scans)
{
  std::vector<ScanMatchJob> jobs;
  if (waypoint_scans.size() < 3U) {
    return jobs;
  }

  std::vector<std::shared_ptr<const ScanBundle>> bundles;
  bundles.reserve(waypoint_scans.size());
  for (const auto & waypoint_scan : waypoint_scans) {
    bundles.push_back(std::make_shared<ScanBundle>(waypoint_scan.bundle));
  }

  jobs.reserve(waypoint_scans.size() - 2U);
  for (std::size_t i = 0; i + 2U < waypoint_scans.size(); ++i) {
    jobs.push_back(ScanMatchJob{
      waypoint_scans[i].index,
      waypoint_scans[i + 2U].index,
      bundles[i],
      bundles[i + 2U]});
  }
  return jobs;
}

bool hasGraphEdge(
  const std::vector<ScanMatchJobResult> & edges,
  std::size_t target_index,
  std::size_t source_index)
{
  return std::any_of(
    edges.begin(), edges.end(),
    [&](const ScanMatchJobResult & edge) {
      return edge.target_index == target_index && edge.source_index == source_index;
    });
}

std::vector<ScanMatchJob> makeLoopClosureScanMatchJobs(
  const std::vector<WaypointScan> & waypoint_scans,
  const std::vector<OptimizedPose2D> & poses,
  const std::vector<ScanMatchJobResult> & existing_edges,
  double max_distance_m,
  std::size_t min_index_separation)
{
  std::vector<std::shared_ptr<const ScanBundle>> bundles;
  bundles.reserve(waypoint_scans.size());
  for (const auto & waypoint_scan : waypoint_scans) {
    bundles.push_back(std::make_shared<ScanBundle>(waypoint_scan.bundle));
  }

  std::vector<ScanMatchJob> jobs;
  for (std::size_t i = 0; i < poses.size(); ++i) {
    for (std::size_t j = i + 1U; j < poses.size(); ++j) {
      const auto separation = poses[j].index - poses[i].index;
      if (separation <= min_index_separation) {
        continue;
      }
      if (hasGraphEdge(existing_edges, poses[i].index, poses[j].index)) {
        continue;
      }

      const double distance = std::hypot(
        poses[j].pose.x - poses[i].pose.x,
        poses[j].pose.y - poses[i].pose.y);
      if (distance > max_distance_m) {
        continue;
      }

      jobs.push_back(ScanMatchJob{
        poses[i].index,
        poses[j].index,
        bundles[poses[i].index],
        bundles[poses[j].index]});
    }
  }
  return jobs;
}

std::vector<ScanMatchJobResult> runCenteredScanMatchJobsParallel(
  const std::vector<ScanMatchJob> & jobs,
  const std::vector<OptimizedPose2D> & initial_poses,
  const ScanMatcherOptions & base_options,
  std::size_t thread_count = 0)
{
  const std::size_t hardware_threads = std::max(1U, std::thread::hardware_concurrency());
  const std::size_t worker_count = std::max<std::size_t>(
    1U, std::min(thread_count == 0U ? hardware_threads : thread_count, jobs.size()));
  std::atomic<std::size_t> next_job{0U};
  std::mutex results_mutex;
  std::vector<ScanMatchJobResult> results;
  results.reserve(jobs.size());

  auto worker = [&]() {
      while (true) {
        const std::size_t job_index = next_job.fetch_add(1U);
        if (job_index >= jobs.size()) {
          break;
        }

        const auto & job = jobs[job_index];
        const auto target_pose = std::find_if(
          initial_poses.begin(), initial_poses.end(),
          [&](const OptimizedPose2D & pose) {
            return pose.index == job.target_index;
          });
        const auto source_pose = std::find_if(
          initial_poses.begin(), initial_poses.end(),
          [&](const OptimizedPose2D & pose) {
            return pose.index == job.source_index;
          });
        if (target_pose == initial_poses.end() || source_pose == initial_poses.end()) {
          continue;
        }

        ScanMatcherOptions options = base_options;
        options.search_center = relativePose(target_pose->pose, source_pose->pose);
        const ScanMatcher matcher(options);
        auto match = matcher.match(job.target->points, job.source->points);
        if (!match.success) {
          continue;
        }

        ScanMatchJobResult result;
        result.target_index = job.target_index;
        result.source_index = job.source_index;
        result.target_bag_path = job.target->bag_path;
        result.source_bag_path = job.source->bag_path;
        result.match = std::move(match);

        std::lock_guard<std::mutex> lock(results_mutex);
        results.push_back(std::move(result));
      }
    };

  std::vector<std::thread> workers;
  workers.reserve(worker_count);
  for (std::size_t i = 0; i < worker_count; ++i) {
    workers.emplace_back(worker);
  }
  for (auto & thread : workers) {
    thread.join();
  }

  std::sort(
    results.begin(), results.end(),
    [](const ScanMatchJobResult & lhs, const ScanMatchJobResult & rhs) {
      if (lhs.target_index != rhs.target_index) {
        return lhs.target_index < rhs.target_index;
      }
      return lhs.source_index < rhs.source_index;
    });
  return results;
}

void writeSimulatedGlobalEndpointPlot(
  const SimulatedLidarPair & pair,
  const std::string & stem)
{
  const auto dir = artifactDirectory();
  const auto target_path = dir / (stem + "_target.dat");
  const auto source_path = dir / (stem + "_source.dat");
  const auto pose_path = dir / (stem + "_poses.dat");
  const auto script_path = dir / (stem + ".gp");
  const auto png_path = dir / (stem + ".png");

  {
    std::ofstream target_data(target_path);
    for (const auto & point : pair.target.endpoints_base.endpoints) {
      const auto transformed = transformPoint(point, pair.target_pose);
      target_data << transformed.x() << ' ' << transformed.y() << '\n';
    }
  }

  {
    std::ofstream source_data(source_path);
    for (const auto & point : pair.source.endpoints_base.endpoints) {
      const auto transformed = transformPoint(point, pair.source_pose);
      source_data << transformed.x() << ' ' << transformed.y() << '\n';
    }
  }

  {
    std::ofstream pose_data(pose_path);
    pose_data << pair.target_pose.x << ' ' << pair.target_pose.y << ' '
              << 0.2 * std::cos(pair.target_pose.yaw) << ' '
              << 0.2 * std::sin(pair.target_pose.yaw) << '\n';
    pose_data << pair.source_pose.x << ' ' << pair.source_pose.y << ' '
              << 0.2 * std::cos(pair.source_pose.yaw) << ' '
              << 0.2 * std::sin(pair.source_pose.yaw) << '\n';
  }

  {
    std::ofstream script(script_path);
    script << "set terminal pngcairo size 1000,800\n";
    script << "set output '" << png_path.string() << "'\n";
    script << "set size ratio -1\n";
    script << "set title 'simulated asymmetric scan pair in global frame'\n";
    script << "set xlabel 'x (m)'\n";
    script << "set ylabel 'y (m)'\n";
    script << "set key outside\n";
    script << "plot '" << target_path.string()
           << "' using 1:2 with points pt 7 ps 0.45 lc rgb '#2ca02c' title 'target scan', \\\n";
    script << "     '" << source_path.string()
           << "' using 1:2 with points pt 7 ps 0.45 lc rgb '#d62728' title 'source scan', \\\n";
    script << "     '" << pose_path.string()
           << "' using 1:2:3:4 with vectors head filled lw 2 lc rgb '#1f77b4' title 'poses'\n";
  }

  runGnuplotScript(script_path);
}

std::filesystem::path mdcDataDirectory(const std::string & dataset_name)
{
  return std::filesystem::path(MAP_SOLVER_SOURCE_DIR) / "data" / dataset_name;
}

TEST(SimPipelineTest, GeneratesNearbyAsymmetricScanPair)
{
  SearchOptions options;
  options.sigma_r = 0.03;
  options.sigma_theta = 0.005;
  const auto pair = simulateNearbyAsymmetricLidarPair(options);
  writeSimulatedGlobalEndpointPlot(pair, "sim_asymmetric_scan_pair_global");

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

TEST(GazeboDataScanMatcherTest, AlignsWaypoint0000And0001)
{
  const std::string mdc_dataset = "mdc_20260520_135327";
  const auto mdc_dir = mdcDataDirectory(mdc_dataset);
  if (!std::filesystem::exists(mdc_dir)) {
    GTEST_SKIP() << "mdc data directory not present: " << mdc_dataset;
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
    "gazebo_waypoint_0000_0001_scan_match_cost");
  writeAlignedScanPlot(
    bundle_0000.points,
    bundle_0001.points,
    result,
    "real scan alignment in waypoint 0000 frame",
    "waypoint 0000",
    "waypoint 0001 transformed",
    "gazebo_waypoint_0000_0001_aligned_scans");

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

TEST(GazeboDataParallelScanMatcherTest, ProcessesSequentialWaypointPairs)
{
  const std::string mdc_dataset = "mdc_20260520_135327";
  const auto mdc_dir = mdcDataDirectory(mdc_dataset);
  if (!std::filesystem::exists(mdc_dir)) {
    GTEST_SKIP() << "mdc data directory not present: " << mdc_dataset;
  }

  ScanBagConversionOptions conversion_options;
  conversion_options.scan_stride = 3;
  conversion_options.sigma_r = 0.03;
  conversion_options.sigma_theta = 0.005;

  const auto waypoint_scans = loadWaypointScans(mdc_dir, conversion_options);
  ASSERT_EQ(waypoint_scans.size(), 11U);
  for (const auto & waypoint_scan : waypoint_scans) {
    EXPECT_FALSE(waypoint_scan.bundle.bag_path.empty());
    EXPECT_TRUE(std::filesystem::exists(waypoint_scan.bundle.bag_path));
    EXPECT_GT(waypoint_scan.bundle.scan_count, 0U);
    EXPECT_GT(waypoint_scan.bundle.points.endpoints.size(), 100U);
  }

  const auto jobs = makeSequentialScanMatchJobs(waypoint_scans);
  ASSERT_EQ(jobs.size(), 10U);

  ScanMatcherOptions match_options;
  match_options.k_neighbors = 5;
  match_options.max_translation = 1.5;
  match_options.coarse_xy_step = 0.25;
  match_options.coarse_yaw_step = 5.0 * kPi / 180.0;
  match_options.refine_levels = 5;
  match_options.max_accepted_cost_per_point = 200.0;

  const auto results = runScanMatchJobsParallel(jobs, match_options);
  ASSERT_EQ(results.size(), jobs.size());
  for (std::size_t i = 0; i < results.size(); ++i) {
    const auto & result = results[i];
    EXPECT_EQ(result.target_index, i);
    EXPECT_EQ(result.source_index, i + 1U);
    EXPECT_TRUE(std::filesystem::exists(result.target_bag_path));
    EXPECT_TRUE(std::filesystem::exists(result.source_bag_path));
    EXPECT_TRUE(result.match.success);
    EXPECT_TRUE(result.match.covariance.allFinite());
    std::cout << "parallel pair " << result.target_index << "->" << result.source_index
              << " x=" << result.match.target_from_source.x
              << " y=" << result.match.target_from_source.y
              << " yaw=" << result.match.target_from_source.yaw
              << " cost_per_point=" << result.match.diagnostics.cost_per_point
              << std::endl;
  }
}

TEST(GazeboDataPoseGraphTest, OptimizesSequentialWaypointPairs)
{
  const std::string mdc_dataset = "mdc_20260520_135327";
  const auto mdc_dir = mdcDataDirectory(mdc_dataset);
  if (!std::filesystem::exists(mdc_dir)) {
    GTEST_SKIP() << "mdc data directory not present: " << mdc_dataset;
  }

  ScanBagConversionOptions conversion_options;
  conversion_options.scan_stride = 3;
  conversion_options.sigma_r = 0.03;
  conversion_options.sigma_theta = 0.005;

  const auto waypoint_scans = loadWaypointScans(mdc_dir, conversion_options);
  ASSERT_EQ(waypoint_scans.size(), 11U);
  const auto jobs = makeSequentialScanMatchJobs(waypoint_scans);
  ASSERT_EQ(jobs.size(), 10U);

  ScanMatcherOptions match_options;
  match_options.k_neighbors = 5;
  match_options.max_translation = 1.5;
  match_options.coarse_xy_step = 0.25;
  match_options.coarse_yaw_step = 5.0 * kPi / 180.0;
  match_options.refine_levels = 5;
  match_options.max_accepted_cost_per_point = 200.0;

  const auto edges = runScanMatchJobsParallel(jobs, match_options);
  ASSERT_EQ(edges.size(), jobs.size());

  PoseGraphOptions graph_options;
  graph_options.prior_sigma_xy = 0.01;
  graph_options.prior_sigma_yaw = 0.01;
  const auto poses = optimizePoseGraphFromScanMatches(edges, graph_options);
  ASSERT_EQ(poses.size(), waypoint_scans.size());
  ASSERT_EQ(poses.front().index, 0U);
  EXPECT_NEAR(poses.front().pose.x, 0.0, 1.0e-9);
  EXPECT_NEAR(poses.front().pose.y, 0.0, 1.0e-9);
  EXPECT_NEAR(poses.front().pose.yaw, 0.0, 1.0e-9);

  writeOptimizedPosePlot(poses, "gazebo_waypoint_pose_graph");

  for (const auto & pose : poses) {
    EXPECT_TRUE(std::isfinite(pose.pose.x));
    EXPECT_TRUE(std::isfinite(pose.pose.y));
    EXPECT_TRUE(std::isfinite(pose.pose.yaw));
    std::cout << "optimized pose " << pose.index
              << " x=" << pose.pose.x
              << " y=" << pose.pose.y
              << " yaw=" << pose.pose.yaw
              << std::endl;
  }
}

TEST(GazeboDataOccupancyMapTest, ReconstructsMapFromOptimizedPoses)
{
  const std::string mdc_dataset = "mdc_20260520_135327";
  const auto mdc_dir = mdcDataDirectory(mdc_dataset);
  if (!std::filesystem::exists(mdc_dir)) {
    GTEST_SKIP() << "mdc data directory not present: " << mdc_dataset;
  }

  ScanBagConversionOptions conversion_options;
  conversion_options.scan_stride = 3;
  conversion_options.sigma_r = 0.03;
  conversion_options.sigma_theta = 0.005;

  const auto waypoint_scans = loadWaypointScans(mdc_dir, conversion_options);
  ASSERT_EQ(waypoint_scans.size(), 11U);
  const auto jobs = makeSequentialScanMatchJobs(waypoint_scans);
  ASSERT_EQ(jobs.size(), 10U);

  ScanMatcherOptions match_options;
  match_options.k_neighbors = 5;
  match_options.max_translation = 1.5;
  match_options.coarse_xy_step = 0.25;
  match_options.coarse_yaw_step = 5.0 * kPi / 180.0;
  match_options.refine_levels = 5;
  match_options.max_accepted_cost_per_point = 200.0;

  const auto edges = runScanMatchJobsParallel(jobs, match_options);
  ASSERT_EQ(edges.size(), jobs.size());

  PoseGraphOptions graph_options;
  graph_options.prior_sigma_xy = 0.01;
  graph_options.prior_sigma_yaw = 0.01;
  const auto poses = optimizePoseGraphFromScanMatches(edges, graph_options);
  ASSERT_EQ(poses.size(), waypoint_scans.size());

  ScanBagConversionOptions full_resolution_options = conversion_options;
  full_resolution_options.scan_stride = 1;
  const auto full_resolution_waypoint_scans =
    loadWaypointScans(mdc_dir, full_resolution_options);
  ASSERT_EQ(full_resolution_waypoint_scans.size(), waypoint_scans.size());

  OccupancyMapOptions map_options;
  map_options.resolution = 0.02;
  map_options.map_padding_meters = 1.0;
  map_options.p_free = 0.35;
  map_options.p_occ = 0.7;
  const auto map = buildOccupancyMap(full_resolution_waypoint_scans, poses, map_options);

  ASSERT_GT(map.width, 0U);
  ASSERT_GT(map.height, 0U);
  ASSERT_EQ(map.occupancy_probability.size(), static_cast<std::size_t>(map.width) * map.height);
  writeOccupancyMapPlot(
    map,
    poses,
    "reconstructed occupancy map",
    "gazebo_waypoint_occupancy_map");

  std::cout << "occupancy map: width=" << map.width
            << " height=" << map.height
            << " resolution=" << map.resolution
            << " origin_x=" << map.origin_x
            << " origin_y=" << map.origin_y
            << " scan_match_stride=" << conversion_options.scan_stride
            << " map_stride=" << full_resolution_options.scan_stride
            << std::endl;
}

TEST(GazeboDataGlobalEndpointPlotTest, PlotsEndpointsInOptimizedFrame)
{
  const std::string mdc_dataset = "mdc_20260520_135327";
  const auto mdc_dir = mdcDataDirectory(mdc_dataset);
  if (!std::filesystem::exists(mdc_dir)) {
    GTEST_SKIP() << "mdc data directory not present: " << mdc_dataset;
  }

  ScanBagConversionOptions conversion_options;
  conversion_options.scan_stride = 3;
  conversion_options.sigma_r = 0.03;
  conversion_options.sigma_theta = 0.005;

  const auto waypoint_scans = loadWaypointScans(mdc_dir, conversion_options);
  ASSERT_EQ(waypoint_scans.size(), 11U);
  const auto jobs = makeSequentialScanMatchJobs(waypoint_scans);
  ASSERT_EQ(jobs.size(), 10U);

  ScanMatcherOptions match_options;
  match_options.k_neighbors = 5;
  match_options.max_translation = 1.5;
  match_options.coarse_xy_step = 0.25;
  match_options.coarse_yaw_step = 5.0 * kPi / 180.0;
  match_options.refine_levels = 5;
  match_options.max_accepted_cost_per_point = 200.0;

  const auto edges = runScanMatchJobsParallel(jobs, match_options);
  ASSERT_EQ(edges.size(), jobs.size());

  PoseGraphOptions graph_options;
  graph_options.prior_sigma_xy = 0.01;
  graph_options.prior_sigma_yaw = 0.01;
  const auto poses = optimizePoseGraphFromScanMatches(edges, graph_options);
  ASSERT_EQ(poses.size(), waypoint_scans.size());

  writeGlobalEndpointPlot(
    waypoint_scans,
    poses,
    "global lidar endpoints after GTSAM correction",
    "gazebo_waypoint_global_endpoints");

  std::size_t endpoint_count = 0;
  for (const auto & waypoint_scan : waypoint_scans) {
    endpoint_count += waypoint_scan.bundle.points.endpoints.size();
  }
  EXPECT_GT(endpoint_count, 1000U);
  std::cout << "global endpoint plot point_count=" << endpoint_count << std::endl;
}

TEST(GazeboDataRawScanMatchEndpointPlotTest, PlotsEndpointsBeforeGtsamAdjustment)
{
  //const std::string mdc_dataset = "mdc_20260520_135327";
  const std::string mdc_dataset = "mdc_20260526_213744";
  const auto mdc_dir = mdcDataDirectory(mdc_dataset);
  if (!std::filesystem::exists(mdc_dir)) {
    GTEST_SKIP() << "mdc data directory not present: " << mdc_dataset;
  }

  ScanBagConversionOptions conversion_options;
  conversion_options.scan_stride = 2;
  conversion_options.sigma_r = 0.03;
  conversion_options.sigma_theta = 0.005;

  const auto waypoint_scans = loadWaypointScans(mdc_dir, conversion_options);
  //ASSERT_EQ(waypoint_scans.size(), 11U);
  const auto jobs = makeSequentialScanMatchJobs(waypoint_scans);
  //ASSERT_EQ(jobs.size(), 10U);

  ScanMatcherOptions match_options;
  match_options.k_neighbors = 5;
  match_options.max_translation = 1.5;
  match_options.coarse_xy_step = 0.25;
  match_options.coarse_yaw_step = 5.0 * kPi / 180.0;
  match_options.refine_levels = 7;
  match_options.max_accepted_cost_per_point = 20000.0;

  const auto edges = runScanMatchJobsParallel(jobs, match_options);
  ASSERT_EQ(edges.size(), jobs.size());

  const auto raw_poses = composeRawSequentialScanMatchPoses(edges);
  ASSERT_EQ(raw_poses.size(), waypoint_scans.size());

  writeGlobalEndpointPlot(
    waypoint_scans,
    raw_poses,
    "global lidar endpoints from raw chained scan matches",
    "gazebo_waypoint_raw_scan_match_global_endpoints");

  std::size_t endpoint_count = 0;
  for (const auto & waypoint_scan : waypoint_scans) {
    endpoint_count += waypoint_scan.bundle.points.endpoints.size();
  }
  EXPECT_GT(endpoint_count, 1000U);
  std::cout << "raw scan-match endpoint plot point_count=" << endpoint_count << std::endl;
}

TEST(GazeboDataLongRangeScanMatcherTest, MatchesFirstAndLastAboutChainedTransform)
{
  const std::string mdc_dataset = "mdc_20260520_135327";
  const auto mdc_dir = mdcDataDirectory(mdc_dataset);
  if (!std::filesystem::exists(mdc_dir)) {
    GTEST_SKIP() << "mdc data directory not present: " << mdc_dataset;
  }

  ScanBagConversionOptions conversion_options;
  conversion_options.scan_stride = 2;
  conversion_options.sigma_r = 0.03;
  conversion_options.sigma_theta = 0.005;

  const auto waypoint_scans = loadWaypointScans(mdc_dir, conversion_options);
  ASSERT_EQ(waypoint_scans.size(), 11U);
  const auto jobs = makeSequentialScanMatchJobs(waypoint_scans);
  ASSERT_EQ(jobs.size(), 10U);

  ScanMatcherOptions adjacent_options;
  adjacent_options.k_neighbors = 10;
  adjacent_options.max_translation = 1.5;
  adjacent_options.coarse_xy_step = 0.25;
  adjacent_options.coarse_yaw_step = 5.0 * kPi / 180.0;
  adjacent_options.refine_levels = 7;
  adjacent_options.max_accepted_cost_per_point = 20000.0;

  const auto adjacent_edges = runScanMatchJobsParallel(jobs, adjacent_options);
  ASSERT_EQ(adjacent_edges.size(), jobs.size());
  const auto raw_poses = composeRawSequentialScanMatchPoses(adjacent_edges);
  ASSERT_EQ(raw_poses.size(), waypoint_scans.size());
  const Pose2D chained_first_from_last = raw_poses.back().pose;

  ScanMatcherOptions long_range_options;
  long_range_options.k_neighbors = 10;
  long_range_options.search_center = chained_first_from_last;
  long_range_options.max_translation = 1.0;
  long_range_options.coarse_yaw_half_width = 20.0 * kPi / 180.0;
  long_range_options.coarse_xy_step = 0.10;
  long_range_options.coarse_yaw_step = 2.0 * kPi / 180.0;
  long_range_options.refine_levels = 7;
  long_range_options.max_accepted_cost_per_point = 20000.0;

  const ScanMatcher matcher(long_range_options);
  const auto result = matcher.match(
    waypoint_scans.front().bundle.points,
    waypoint_scans.back().bundle.points);

  writeScanMatchCostMap(
    waypoint_scans.back().bundle.points,
    std::nullopt,
    result,
    "gazebo_waypoint_0000_0010_long_range_scan_match_cost");
  writeAlignedScanPlot(
    waypoint_scans.front().bundle.points,
    waypoint_scans.back().bundle.points,
    result,
    "long-range scan alignment in waypoint 0000 frame",
    "waypoint 0000",
    "waypoint 0010 transformed",
    "gazebo_waypoint_0000_0010_long_range_aligned_scans");

  const double translation_delta = std::hypot(
    result.target_from_source.x - chained_first_from_last.x,
    result.target_from_source.y - chained_first_from_last.y);
  const double yaw_delta = yawError(result.target_from_source.yaw, chained_first_from_last.yaw);

  std::cout << "chained first_from_last: x=" << chained_first_from_last.x
            << " y=" << chained_first_from_last.y
            << " yaw=" << chained_first_from_last.yaw << std::endl;
  std::cout << "long-range result: x=" << result.target_from_source.x
            << " y=" << result.target_from_source.y
            << " yaw=" << result.target_from_source.yaw
            << " cost_per_point=" << result.diagnostics.cost_per_point
            << " translation_delta=" << translation_delta
            << " yaw_delta=" << yaw_delta
            << std::endl;

  ASSERT_TRUE(result.success) << result.message
                              << " cost_per_point=" << result.diagnostics.cost_per_point;
  EXPECT_LE(translation_delta, 1.0);
  EXPECT_LE(yaw_delta, 20.0 * kPi / 180.0);
}

TEST(GazeboDataSecondNeighborPoseGraphTest, OptimizesWithNearestAndSecondNearestEdges)
{
  //const std::string mdc_dataset = "mdc_20260520_135327";
  const std::string mdc_dataset = "mdc_20260526_213744";
  const auto mdc_dir = mdcDataDirectory(mdc_dataset);
  if (!std::filesystem::exists(mdc_dir)) {
    GTEST_SKIP() << "mdc data directory not present: " << mdc_dataset;
  }

  ScanBagConversionOptions conversion_options;
  conversion_options.scan_stride = 2;
  conversion_options.sigma_r = 0.03;
  conversion_options.sigma_theta = 0.005;

  const auto waypoint_scans = loadWaypointScans(mdc_dir, conversion_options);
  //ASSERT_EQ(waypoint_scans.size(), 11U);

  const auto nn_jobs = makeSequentialScanMatchJobs(waypoint_scans);
  //ASSERT_EQ(nn_jobs.size(), 10U);

  ScanMatcherOptions nn_options;
  nn_options.k_neighbors = 5;
  nn_options.max_translation = 1.5;
  nn_options.coarse_xy_step = 0.25;
  nn_options.coarse_yaw_step = 5.0 * kPi / 180.0;
  nn_options.refine_levels = 7;
  nn_options.max_accepted_cost_per_point = 20000.0;

  const auto nn_edges = runScanMatchJobsParallel(nn_jobs, nn_options);
  ASSERT_EQ(nn_edges.size(), nn_jobs.size());

  const auto raw_poses = composeRawSequentialScanMatchPoses(nn_edges);
  ASSERT_EQ(raw_poses.size(), waypoint_scans.size());

  const auto second_nn_jobs = makeSkipOneScanMatchJobs(waypoint_scans);
  //ASSERT_EQ(second_nn_jobs.size(), 9U);

  ScanMatcherOptions second_nn_options;
  second_nn_options.k_neighbors = 5;
  second_nn_options.max_translation = 1.0;
  second_nn_options.coarse_yaw_half_width = 20.0 * kPi / 180.0;
  second_nn_options.coarse_xy_step = 0.10;
  second_nn_options.coarse_yaw_step = 2.0 * kPi / 180.0;
  second_nn_options.refine_levels = 7;
  second_nn_options.max_accepted_cost_per_point = 20000.0;

  const auto second_nn_edges =
    runCenteredScanMatchJobsParallel(second_nn_jobs, raw_poses, second_nn_options);
  ASSERT_EQ(second_nn_edges.size(), second_nn_jobs.size());

  std::vector<ScanMatchJobResult> graph_edges = nn_edges;
  graph_edges.insert(graph_edges.end(), second_nn_edges.begin(), second_nn_edges.end());

  PoseGraphOptions graph_options;
  graph_options.prior_sigma_xy = 0.01;
  graph_options.prior_sigma_yaw = 0.01;
  const auto poses = optimizePoseGraphFromScanMatches(graph_edges, graph_options);
  ASSERT_EQ(poses.size(), waypoint_scans.size());

  writeGlobalEndpointPlot(
    waypoint_scans,
    poses,
    "global lidar endpoints after GTSAM with NN and 2nd-NN edges",
    "gazebo_waypoint_nn_2nn_gtsam_global_endpoints");
  writeOptimizedPosePlot(poses, "gazebo_waypoint_nn_2nn_pose_graph");

  for (const auto & edge : second_nn_edges) {
    std::cout << "2nd-NN edge " << edge.target_index << "->" << edge.source_index
              << " x=" << edge.match.target_from_source.x
              << " y=" << edge.match.target_from_source.y
              << " yaw=" << edge.match.target_from_source.yaw
              << " cost_per_point=" << edge.match.diagnostics.cost_per_point
              << std::endl;
  }
}

TEST(GazeboDataSecondNeighborPoseGraphTest, OptimizesWithLoopClosure)
{
  //const std::string mdc_dataset = "mdc_20260520_135327";
  const std::string mdc_dataset = "mdc_20260526_213744";
  const auto mdc_dir = mdcDataDirectory(mdc_dataset);
  if (!std::filesystem::exists(mdc_dir)) {
    GTEST_SKIP() << "mdc data directory not present: " << mdc_dataset;
  }

  ScanBagConversionOptions conversion_options;
  conversion_options.scan_stride = 2;
  conversion_options.sigma_r = 0.03;
  conversion_options.sigma_theta = 0.005;

  const auto waypoint_scans = loadWaypointScans(mdc_dir, conversion_options);
  const auto nn_jobs = makeSequentialScanMatchJobs(waypoint_scans);

  ScanMatcherOptions nn_options;
  nn_options.k_neighbors = 5;
  nn_options.max_translation = 1.5;
  nn_options.coarse_xy_step = 0.25;
  nn_options.coarse_yaw_step = 5.0 * kPi / 180.0;
  nn_options.refine_levels = 7;
  nn_options.max_accepted_cost_per_point = 20000.0;

  const auto nn_edges = runScanMatchJobsParallel(nn_jobs, nn_options);
  ASSERT_EQ(nn_edges.size(), nn_jobs.size());

  const auto raw_poses = composeRawSequentialScanMatchPoses(nn_edges);
  ASSERT_EQ(raw_poses.size(), waypoint_scans.size());

  const auto second_nn_jobs = makeSkipOneScanMatchJobs(waypoint_scans);

  ScanMatcherOptions second_nn_options;
  second_nn_options.k_neighbors = 5;
  second_nn_options.max_translation = 1.0;
  second_nn_options.coarse_yaw_half_width = 20.0 * kPi / 180.0;
  second_nn_options.coarse_xy_step = 0.10;
  second_nn_options.coarse_yaw_step = 2.0 * kPi / 180.0;
  second_nn_options.refine_levels = 7;
  second_nn_options.max_accepted_cost_per_point = 20000.0;

  const auto second_nn_edges =
    runCenteredScanMatchJobsParallel(second_nn_jobs, raw_poses, second_nn_options);
  ASSERT_EQ(second_nn_edges.size(), second_nn_jobs.size());

  std::vector<ScanMatchJobResult> graph_edges = nn_edges;
  graph_edges.insert(graph_edges.end(), second_nn_edges.begin(), second_nn_edges.end());

  PoseGraphOptions graph_options;
  graph_options.prior_sigma_xy = 0.01;
  graph_options.prior_sigma_yaw = 0.01;
  const auto first_pass_poses = optimizePoseGraphFromScanMatches(graph_edges, graph_options);
  ASSERT_EQ(first_pass_poses.size(), waypoint_scans.size());

  const auto loop_closure_jobs = makeLoopClosureScanMatchJobs(
    waypoint_scans,
    first_pass_poses,
    graph_edges,
    1.0,
    5U);
  std::cout << "loop closure candidates=" << loop_closure_jobs.size() << std::endl;

  ScanMatcherOptions loop_closure_options;
  loop_closure_options.k_neighbors = 5;
  loop_closure_options.max_translation = 1.0;
  loop_closure_options.coarse_yaw_half_width = 20.0 * kPi / 180.0;
  loop_closure_options.coarse_xy_step = 0.10;
  loop_closure_options.coarse_yaw_step = 2.0 * kPi / 180.0;
  loop_closure_options.refine_levels = 7;
  loop_closure_options.max_accepted_cost_per_point = 20000.0;

  const auto loop_closure_edges =
    runCenteredScanMatchJobsParallel(loop_closure_jobs, first_pass_poses, loop_closure_options);
  std::cout << "loop closures accepted=" << loop_closure_edges.size() << std::endl;
  ASSERT_GE(loop_closure_edges.size(), 1U);

  graph_edges.insert(graph_edges.end(), loop_closure_edges.begin(), loop_closure_edges.end());
  const auto loop_closed_poses = optimizePoseGraphFromScanMatches(graph_edges, graph_options);
  ASSERT_EQ(loop_closed_poses.size(), waypoint_scans.size());

  writeGlobalEndpointPlot(
    waypoint_scans,
    loop_closed_poses,
    "global lidar endpoints after GTSAM with loop closures",
    "gazebo_waypoint_loop_closure_gtsam_global_endpoints");
  writeOptimizedPosePlot(loop_closed_poses, "gazebo_waypoint_loop_closure_pose_graph");

  ScanBagConversionOptions full_resolution_options = conversion_options;
  full_resolution_options.scan_stride = 1;
  const auto full_resolution_waypoint_scans =
    loadWaypointScans(mdc_dir, full_resolution_options);
  ASSERT_EQ(full_resolution_waypoint_scans.size(), waypoint_scans.size());

  OccupancyMapOptions map_options;
  map_options.resolution = 0.02;
  map_options.map_padding_meters = 1.0;
  map_options.p_free = 0.35;
  map_options.p_occ = 0.7;
  const auto loop_closure_map = buildOccupancyMap(
    full_resolution_waypoint_scans, loop_closed_poses, map_options);
  ASSERT_GT(loop_closure_map.width, 0U);
  ASSERT_GT(loop_closure_map.height, 0U);
  writeOccupancyMapPlot(
    loop_closure_map,
    loop_closed_poses,
    "reconstructed occupancy map after loop closure",
    "gazebo_waypoint_loop_closure_occupancy_map");

  for (const auto & edge : loop_closure_edges) {
    std::cout << "loop closure edge " << edge.target_index << "->" << edge.source_index
              << " x=" << edge.match.target_from_source.x
              << " y=" << edge.match.target_from_source.y
              << " yaw=" << edge.match.target_from_source.yaw
              << " cost_per_point=" << edge.match.diagnostics.cost_per_point
              << std::endl;
  }
}

}  // namespace
}  // namespace map_solver
