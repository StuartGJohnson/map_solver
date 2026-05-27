#include "map_solver/pose_graph.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Eigenvalues>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace map_solver
{
namespace
{

gtsam::Pose2 toGtsamPose(const Pose2D & pose)
{
  return gtsam::Pose2(pose.x, pose.y, pose.yaw);
}

Pose2D fromGtsamPose(const gtsam::Pose2 & pose)
{
  return Pose2D{pose.x(), pose.y(), normalizeYaw(pose.theta())};
}

Eigen::Matrix3d regularizedCovariance(
  const Eigen::Matrix3d & covariance,
  double regularization)
{
  Eigen::Matrix3d symmetric = 0.5 * (covariance + covariance.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(symmetric);
  if (solver.info() != Eigen::Success) {
    return Eigen::Matrix3d::Identity() / regularization;
  }

  Eigen::Vector3d eigenvalues = solver.eigenvalues();
  for (int i = 0; i < 3; ++i) {
    eigenvalues(i) = std::max(eigenvalues(i), regularization);
  }
  return solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
}

std::size_t maxPoseIndex(const std::vector<ScanMatchJobResult> & edges)
{
  std::size_t max_index = 0;
  for (const auto & edge : edges) {
    max_index = std::max(max_index, edge.target_index);
    max_index = std::max(max_index, edge.source_index);
  }
  return max_index;
}

}  // namespace

std::vector<OptimizedPose2D> optimizePoseGraphFromScanMatches(
  const std::vector<ScanMatchJobResult> & edges,
  const PoseGraphOptions & options)
{
  if (edges.empty()) {
    return {};
  }

  const std::size_t max_index = maxPoseIndex(edges);
  gtsam::NonlinearFactorGraph graph;

  const auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(3) << options.prior_sigma_xy, options.prior_sigma_xy,
      options.prior_sigma_yaw).finished());
  graph.add(gtsam::PriorFactor<gtsam::Pose2>(0U, gtsam::Pose2(0.0, 0.0, 0.0), prior_noise));

  for (const auto & edge : edges) {
    if (!edge.match.success) {
      continue;
    }
    const auto covariance = regularizedCovariance(
      edge.match.covariance, options.covariance_regularization);
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
      edge.target_index,
      edge.source_index,
      toGtsamPose(edge.match.target_from_source),
      gtsam::noiseModel::Gaussian::Covariance(covariance)));
  }

  gtsam::Values initial;
  initial.insert(0U, gtsam::Pose2(0.0, 0.0, 0.0));
  for (const auto & edge : edges) {
    if (!edge.match.success || !initial.exists(edge.target_index) ||
      initial.exists(edge.source_index))
    {
      continue;
    }
    const auto target_pose = initial.at<gtsam::Pose2>(edge.target_index);
    initial.insert(
      edge.source_index,
      target_pose.compose(toGtsamPose(edge.match.target_from_source)));
  }

  for (std::size_t i = 0; i <= max_index; ++i) {
    if (!initial.exists(i)) {
      initial.insert(i, gtsam::Pose2(0.0, 0.0, 0.0));
    }
  }

  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("SILENT");
  const gtsam::Values optimized =
    gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
  const gtsam::Marginals marginals(graph, optimized);

  std::vector<OptimizedPose2D> poses;
  poses.reserve(max_index + 1U);
  for (std::size_t i = 0; i <= max_index; ++i) {
    if (optimized.exists(i)) {
      poses.push_back(OptimizedPose2D{
        i,
        fromGtsamPose(optimized.at<gtsam::Pose2>(i)),
        marginals.marginalCovariance(i)});
    }
  }
  return poses;
}

}  // namespace map_solver
