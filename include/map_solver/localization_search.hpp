#ifndef MAP_SOLVER__LOCALIZATION_SEARCH_HPP_
#define MAP_SOLVER__LOCALIZATION_SEARCH_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "map_solver/map_model.hpp"

namespace map_solver
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

inline double normalizeYaw(double yaw)
{
  while (yaw >= kPi) {
    yaw -= kTwoPi;
  }
  while (yaw < -kPi) {
    yaw += kTwoPi;
  }
  return yaw;
}

inline double yawError(double lhs, double rhs)
{
  return std::abs(normalizeYaw(lhs - rhs));
}

struct SearchOptions
{
  double coarse_xy_step{0.5};
  double coarse_yaw_step{0.2617993877991494};
  int top_k{10};
  double candidate_min_xy_separation{0.0};
  double candidate_min_yaw_separation{0.0};
  int refine_levels{3};
  double refine_xy_step{0.2};
  double refine_yaw_step{0.08726646259971647};
  int scan_stride{1};
  double max_range{0.0};
  double off_map_distance{5.0};
  double sigma_r{0.3};
  double sigma_theta{0.005};
  double alpha_m{1.0};
  double map_padding_xy{1.0};
  double free_space_weight{0.0};
  double free_space_sample_step{0.15};
  int min_endpoint_count{20};
  double covariance_step_xy{0.03};
  double covariance_step_yaw{0.017453292519943295};
  double covariance_regularization{1.0e-3};
  double covariance_scale{1.0};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Candidate
{
  Pose2D pose;
  double cost{0.0};
};

struct LaserScanPoints
{
  // endpoints (typically in the robot frame)
  std::vector<Eigen::Vector2d> endpoints;
  // scan range for each point - for obs. variance
  std::vector<double> ranges;
  // the (precomputed) lidar variance for each measurement
  std::vector<double> variances;
};

struct SearchResult
{
  bool success{false};
  std::string message;
  Candidate best;
  std::vector<Candidate> candidates;
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Identity()};
  std::size_t endpoint_count{0};
};

class LocalizationSearch
{
public:
  explicit LocalizationSearch(SearchOptions options);

  void dumpCoarseScore(const std::string& filename);

  SearchResult run(
    const MapModel & map,
    const LaserScanPoints& endpoints);
  double scorePose(
    const MapModel & map,
    const LaserScanPoints& endpoints,
    const Pose2D & pose) const;

private:
  std::vector<Candidate> coarseSearch(
    const MapModel & map,
    const LaserScanPoints& endpoints);
  Candidate refineCandidate(const MapModel & map,
     const LaserScanPoints& endpoints,
     const Candidate & initial) const;
  Eigen::Matrix3d covarianceFromHessian(
    const MapModel & map,
    const LaserScanPoints& endpoints,
    const Pose2D & pose) const;

  // for plotting/grokking
  Eigen::MatrixXd coarse_score;
  std::uint32_t xy_step_cells;
  double resolution;

  SearchOptions options_;
};

}  // namespace map_solver

#endif  // MAP_SOLVER__LOCALIZATION_SEARCH_HPP_
