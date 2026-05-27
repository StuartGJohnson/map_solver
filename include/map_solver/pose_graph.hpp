#ifndef MAP_SOLVER__POSE_GRAPH_HPP_
#define MAP_SOLVER__POSE_GRAPH_HPP_

#include <cstddef>
#include <vector>

#include <Eigen/Dense>

#include "map_solver/localization_search.hpp"
#include "map_solver/scan_match_batch.hpp"

namespace map_solver
{

struct PoseGraphOptions
{
  double prior_sigma_xy{0.01};
  double prior_sigma_yaw{0.01};
  double covariance_regularization{1.0e-6};
};

struct OptimizedPose2D
{
  std::size_t index{0};
  Pose2D pose;
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Identity()};
};

std::vector<OptimizedPose2D> optimizePoseGraphFromScanMatches(
  const std::vector<ScanMatchJobResult> & edges,
  const PoseGraphOptions & options = {});

}  // namespace map_solver

#endif  // MAP_SOLVER__POSE_GRAPH_HPP_
