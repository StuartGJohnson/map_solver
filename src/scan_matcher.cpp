#include "map_solver/scan_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <utility>

#include <Eigen/Eigenvalues>

namespace map_solver
{
namespace
{

struct Neighbor
{
  double squared_distance{0.0};
  std::size_t index{0};
};

class KdTree2d
{
public:
  explicit KdTree2d(const std::vector<Eigen::Vector2d> & points)
  : points_(points)
  {
    indices_.resize(points_.size());
    std::iota(indices_.begin(), indices_.end(), 0U);
    root_ = build(0, indices_.size(), 0);
  }

  std::vector<Neighbor> nearest(const Eigen::Vector2d & query, int k) const
  {
    std::priority_queue<std::pair<double, std::size_t>> heap;
    nearest(root_, query, std::max(k, 1), heap);

    std::vector<Neighbor> out;
    out.reserve(heap.size());
    while (!heap.empty()) {
      out.push_back(Neighbor{heap.top().first, heap.top().second});
      heap.pop();
    }
    std::reverse(out.begin(), out.end());
    return out;
  }

private:
  struct Node
  {
    std::size_t index{0};
    int axis{0};
    int left{-1};
    int right{-1};
  };

  int build(std::size_t begin, std::size_t end, int depth)
  {
    if (begin >= end) {
      return -1;
    }
    const int axis = depth % 2;
    const std::size_t mid = begin + (end - begin) / 2U;
    std::nth_element(
      indices_.begin() + static_cast<std::ptrdiff_t>(begin),
      indices_.begin() + static_cast<std::ptrdiff_t>(mid),
      indices_.begin() + static_cast<std::ptrdiff_t>(end),
      [&](std::size_t lhs, std::size_t rhs) {
        return points_[lhs](axis) < points_[rhs](axis);
      });

    const int node_index = static_cast<int>(nodes_.size());
    nodes_.push_back(Node{indices_[mid], axis, -1, -1});
    nodes_[node_index].left = build(begin, mid, depth + 1);
    nodes_[node_index].right = build(mid + 1U, end, depth + 1);
    return node_index;
  }

  void nearest(
    int node_index,
    const Eigen::Vector2d & query,
    int k,
    std::priority_queue<std::pair<double, std::size_t>> & heap) const
  {
    if (node_index < 0) {
      return;
    }
    const auto & node = nodes_[static_cast<std::size_t>(node_index)];
    const double dist2 = (query - points_[node.index]).squaredNorm();
    if (static_cast<int>(heap.size()) < k) {
      heap.emplace(dist2, node.index);
    } else if (dist2 < heap.top().first) {
      heap.pop();
      heap.emplace(dist2, node.index);
    }

    const double delta = query(node.axis) - points_[node.index](node.axis);
    const int first = delta < 0.0 ? node.left : node.right;
    const int second = delta < 0.0 ? node.right : node.left;
    nearest(first, query, k, heap);
    if (static_cast<int>(heap.size()) < k || delta * delta < heap.top().first) {
      nearest(second, query, k, heap);
    }
  }

  const std::vector<Eigen::Vector2d> & points_;
  std::vector<std::size_t> indices_;
  std::vector<Node> nodes_;
  int root_{-1};
};

double pointVariance(const LaserScanPoints & points, std::size_t index, double default_variance)
{
  if (index < points.variances.size() && std::isfinite(points.variances[index]) &&
    points.variances[index] > 0.0)
  {
    return points.variances[index];
  }
  return default_variance;
}

double stableLogSumExp(const std::vector<double> & values)
{
  const double max_value = *std::max_element(values.begin(), values.end());
  double sum = 0.0;
  for (const double value : values) {
    sum += std::exp(value - max_value);
  }
  return max_value + std::log(sum);
}

Eigen::Vector2d transformPoint(const Eigen::Vector2d & point, const Pose2D & pose)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return Eigen::Vector2d{
    pose.x + c * point.x() - s * point.y(),
    pose.y + s * point.x() + c * point.y()};
}

Eigen::Matrix3d covarianceFromHessian(
  const Eigen::Matrix3d & hessian,
  double regularization,
  double & condition)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
    hessian + regularization * Eigen::Matrix3d::Identity());
  if (solver.info() != Eigen::Success) {
    condition = std::numeric_limits<double>::infinity();
    return Eigen::Matrix3d::Identity() / regularization;
  }

  Eigen::Vector3d eigenvalues = solver.eigenvalues();
  const double min_ev = std::max(eigenvalues.minCoeff(), regularization);
  const double max_ev = std::max(eigenvalues.maxCoeff(), regularization);
  condition = max_ev / min_ev;
  for (int i = 0; i < 3; ++i) {
    eigenvalues(i) = 1.0 / std::max(eigenvalues(i), regularization);
  }
  return solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
}

double scoreWithTree(
  const LaserScanPoints & target,
  const LaserScanPoints & source,
  const Pose2D & target_from_source,
  const KdTree2d & tree,
  const ScanMatcherOptions & options)
{
  double total = 0.0;
  for (std::size_t i = 0; i < source.endpoints.size(); ++i) {
    const auto transformed = transformPoint(source.endpoints[i], target_from_source);
    const auto neighbors = tree.nearest(transformed, options.k_neighbors);
    std::vector<double> terms;
    terms.reserve(neighbors.size());
    const double source_variance = pointVariance(source, i, options.default_variance);
    const double log_weight = -std::log(static_cast<double>(neighbors.size()));
    for (const auto & neighbor : neighbors) {
      const double variance = source_variance +
        pointVariance(target, neighbor.index, options.default_variance);
      terms.push_back(
        log_weight - std::log(2.0 * kPi * variance) -
        neighbor.squared_distance / (2.0 * variance));
    }
    total += -stableLogSumExp(terms);
  }
  return total;
}

}  // namespace

ScanMatcher::ScanMatcher(ScanMatcherOptions options)
: options_(options)
{
  options_.k_neighbors = std::max(options_.k_neighbors, 1);
  options_.max_translation = std::max(options_.max_translation, 0.01);
  options_.coarse_xy_step = std::max(options_.coarse_xy_step, 0.005);
  options_.coarse_yaw_step = std::max(options_.coarse_yaw_step, 0.001);
  options_.coarse_yaw_half_width = std::clamp(
    options_.coarse_yaw_half_width, options_.coarse_yaw_step, kPi);
  options_.refine_levels = std::max(options_.refine_levels, 0);
  options_.min_xy_step = std::max(options_.min_xy_step, 0.001);
  options_.min_yaw_step = std::max(options_.min_yaw_step, 1.0e-4);
  options_.default_variance = std::max(options_.default_variance, 1.0e-9);
  options_.covariance_regularization = std::max(options_.covariance_regularization, 1.0e-9);
}

ScanMatchResult ScanMatcher::match(
  const LaserScanPoints & target,
  const LaserScanPoints & source) const
{
  ScanMatchResult result;
  result.diagnostics.target_point_count = target.endpoints.size();
  result.diagnostics.source_point_count = source.endpoints.size();
  if (target.endpoints.empty() || source.endpoints.empty()) {
    result.message = "target and source scans must both contain points";
    return result;
  }

  const KdTree2d tree(target.endpoints);
  auto score_candidate = [&](const Pose2D & pose) {
      return scoreWithTree(target, source, pose, tree, options_);
    };

  const int x_count = static_cast<int>(
    std::floor((2.0 * options_.max_translation) / options_.coarse_xy_step)) + 1;
  const int y_count = static_cast<int>(
    std::floor((2.0 * options_.max_translation) / options_.coarse_xy_step)) + 1;
  const int yaw_steps = std::max(
    1, static_cast<int>(
      std::floor((2.0 * options_.coarse_yaw_half_width) / options_.coarse_yaw_step)) + 1);
  result.coarse_cost_map.min_x = options_.search_center.x - options_.max_translation;
  result.coarse_cost_map.min_y = options_.search_center.y - options_.max_translation;
  result.coarse_cost_map.xy_step = options_.coarse_xy_step;
  result.coarse_cost_map.min_score_by_xy.resize(x_count, y_count);
  result.coarse_cost_map.min_score_by_xy.setConstant(std::numeric_limits<double>::infinity());

  Pose2D best;
  double best_cost = std::numeric_limits<double>::infinity();
  for (int ix = 0; ix < x_count; ++ix) {
    const double x = result.coarse_cost_map.min_x +
      static_cast<double>(ix) * options_.coarse_xy_step;
    for (int iy = 0; iy < y_count; ++iy) {
      const double y = result.coarse_cost_map.min_y +
        static_cast<double>(iy) * options_.coarse_xy_step;
      for (int yaw_index = 0; yaw_index < yaw_steps; ++yaw_index) {
        const double yaw = options_.search_center.yaw - options_.coarse_yaw_half_width +
          static_cast<double>(yaw_index) * options_.coarse_yaw_step;
        const Pose2D pose{x, y, normalizeYaw(yaw)};
        const double cost = score_candidate(pose);
        result.diagnostics.evaluated_pose_count += 1U;
        if (cost < result.coarse_cost_map.min_score_by_xy(ix, iy)) {
          result.coarse_cost_map.min_score_by_xy(ix, iy) = cost;
        }
        if (cost < best_cost) {
          best = pose;
          best_cost = cost;
        }
      }
    }
  }

  double xy_step = options_.coarse_xy_step * 0.5;
  double yaw_step = options_.coarse_yaw_step * 0.5;
  for (int level = 0; level < options_.refine_levels; ++level) {
    bool improved = true;
    while (improved) {
      improved = false;
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dt = -1; dt <= 1; ++dt) {
            if (dx == 0 && dy == 0 && dt == 0) {
              continue;
            }
            const Pose2D candidate{
              std::clamp(best.x + static_cast<double>(dx) * xy_step,
                options_.search_center.x - options_.max_translation,
                options_.search_center.x + options_.max_translation),
              std::clamp(best.y + static_cast<double>(dy) * xy_step,
                options_.search_center.y - options_.max_translation,
                options_.search_center.y + options_.max_translation),
              normalizeYaw(options_.search_center.yaw + std::clamp(
                normalizeYaw(best.yaw + static_cast<double>(dt) * yaw_step -
                options_.search_center.yaw),
                -options_.coarse_yaw_half_width,
                options_.coarse_yaw_half_width))};
            const double cost = score_candidate(candidate);
            result.diagnostics.evaluated_pose_count += 1U;
            if (cost + 1.0e-12 < best_cost) {
              best = candidate;
              best_cost = cost;
              improved = true;
            }
          }
        }
      }
    }
    xy_step = std::max(0.5 * xy_step, options_.min_xy_step);
    yaw_step = std::max(0.5 * yaw_step, options_.min_yaw_step);
  }

  const Eigen::Vector3d step{
    options_.covariance_step_xy,
    options_.covariance_step_xy,
    options_.covariance_step_yaw};
  Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
  const double f0 = score_candidate(best);
  auto offset_pose = [](Pose2D pose, int index, double delta) {
      if (index == 0) {
        pose.x += delta;
      } else if (index == 1) {
        pose.y += delta;
      } else {
        pose.yaw = normalizeYaw(pose.yaw + delta);
      }
      return pose;
    };

  for (int i = 0; i < 3; ++i) {
    const double fp = score_candidate(offset_pose(best, i, step(i)));
    const double fm = score_candidate(offset_pose(best, i, -step(i)));
    hessian(i, i) = (fp - 2.0 * f0 + fm) / (step(i) * step(i));
    for (int j = i + 1; j < 3; ++j) {
      const double fpp = score_candidate(offset_pose(offset_pose(best, i, step(i)), j, step(j)));
      const double fpm = score_candidate(offset_pose(offset_pose(best, i, step(i)), j, -step(j)));
      const double fmp = score_candidate(offset_pose(offset_pose(best, i, -step(i)), j, step(j)));
      const double fmm = score_candidate(offset_pose(offset_pose(best, i, -step(i)), j, -step(j)));
      hessian(i, j) = (fpp - fpm - fmp + fmm) / (4.0 * step(i) * step(j));
      hessian(j, i) = hessian(i, j);
    }
  }

  result.target_from_source = best;
  result.diagnostics.best_cost = best_cost;
  result.diagnostics.cost_per_point = best_cost / static_cast<double>(source.endpoints.size());
  result.covariance = covarianceFromHessian(
    hessian, options_.covariance_regularization, result.diagnostics.hessian_condition);
  result.information = result.covariance.inverse();
  result.diagnostics.accepted =
    result.diagnostics.cost_per_point <= options_.max_accepted_cost_per_point &&
    result.covariance.allFinite();
  result.success = result.diagnostics.accepted;
  result.message = result.success ? "scan match accepted" : "scan match rejected by diagnostics";
  return result;
}

double ScanMatcher::score(
  const LaserScanPoints & target,
  const LaserScanPoints & source,
  const Pose2D & target_from_source) const
{
  if (target.endpoints.empty() || source.endpoints.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  const KdTree2d tree(target.endpoints);
  return scoreWithTree(target, source, target_from_source, tree, options_);
}

PoseGraphEdge ScanMatcher::makePoseGraphEdge(
  std::size_t target_key,
  std::size_t source_key,
  const ScanMatchResult & result) const
{
  if (!result.success) {
    throw std::runtime_error("cannot create pose graph edge from rejected scan match");
  }
  return PoseGraphEdge{
    target_key,
    source_key,
    result.target_from_source,
    result.covariance,
    result.information,
    result.diagnostics};
}

Pose2D inversePose(const Pose2D & pose)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return Pose2D{
    -c * pose.x - s * pose.y,
    s * pose.x - c * pose.y,
    normalizeYaw(-pose.yaw)};
}

Pose2D composePoses(const Pose2D & lhs, const Pose2D & rhs)
{
  const double c = std::cos(lhs.yaw);
  const double s = std::sin(lhs.yaw);
  return Pose2D{
    lhs.x + c * rhs.x - s * rhs.y,
    lhs.y + s * rhs.x + c * rhs.y,
    normalizeYaw(lhs.yaw + rhs.yaw)};
}

Pose2D relativePose(const Pose2D & target_world_pose, const Pose2D & source_world_pose)
{
  return composePoses(inversePose(target_world_pose), source_world_pose);
}

}  // namespace map_solver
