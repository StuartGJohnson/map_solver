#ifndef MAP_SOLVER__SCAN_MATCHER_HPP_
#define MAP_SOLVER__SCAN_MATCHER_HPP_

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "map_solver/localization_search.hpp"

namespace map_solver
{

struct ScanMatcherOptions
{
  int k_neighbors{5};
  double max_translation{1.0};
  double coarse_xy_step{0.20};
  double coarse_yaw_step{5.0 * kPi / 180.0};
  int refine_levels{4};
  double min_xy_step{0.01};
  double min_yaw_step{0.25 * kPi / 180.0};
  double default_variance{0.0025};
  double covariance_step_xy{0.02};
  double covariance_step_yaw{0.5 * kPi / 180.0};
  double covariance_regularization{1.0e-6};
  double max_accepted_cost_per_point{20.0};
};

struct ScanMatchDiagnostics
{
  double best_cost{std::numeric_limits<double>::infinity()};
  double cost_per_point{std::numeric_limits<double>::infinity()};
  double hessian_condition{std::numeric_limits<double>::infinity()};
  std::size_t source_point_count{0};
  std::size_t target_point_count{0};
  std::size_t evaluated_pose_count{0};
  bool accepted{false};
};

struct ScanMatchCostMap
{
  Eigen::MatrixXd min_score_by_xy;
  double min_x{0.0};
  double min_y{0.0};
  double xy_step{0.0};
};

struct ScanMatchResult
{
  bool success{false};
  std::string message;
  Pose2D target_from_source;
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d information{Eigen::Matrix3d::Identity()};
  ScanMatchDiagnostics diagnostics;
  ScanMatchCostMap coarse_cost_map;
};

struct PoseGraphEdge
{
  std::size_t target_key{0};
  std::size_t source_key{0};
  Pose2D target_from_source;
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d information{Eigen::Matrix3d::Identity()};
  ScanMatchDiagnostics diagnostics;
};

class ScanMatcher
{
public:
  explicit ScanMatcher(ScanMatcherOptions options = {});

  ScanMatchResult match(
    const LaserScanPoints & target,
    const LaserScanPoints & source) const;

  double score(
    const LaserScanPoints & target,
    const LaserScanPoints & source,
    const Pose2D & target_from_source) const;

  PoseGraphEdge makePoseGraphEdge(
    std::size_t target_key,
    std::size_t source_key,
    const ScanMatchResult & result) const;

private:
  ScanMatcherOptions options_;
};

Pose2D inversePose(const Pose2D & pose);
Pose2D composePoses(const Pose2D & lhs, const Pose2D & rhs);
Pose2D relativePose(const Pose2D & target_world_pose, const Pose2D & source_world_pose);

}  // namespace map_solver

#endif  // MAP_SOLVER__SCAN_MATCHER_HPP_
