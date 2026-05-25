#include "map_solver/localization_search.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Eigenvalues>
#include <fstream>
#include <iomanip> // For controlling precision if needed

namespace map_solver
{

LocalizationSearch::LocalizationSearch(SearchOptions options)
: options_(options)
{
  options_.coarse_xy_step = std::max(options_.coarse_xy_step, 1.0e-3);
  options_.coarse_yaw_step = std::max(options_.coarse_yaw_step, 1.0e-3);
  options_.top_k = std::max(options_.top_k, 1);
  options_.candidate_min_xy_separation = std::max(options_.candidate_min_xy_separation, 0.0);
  options_.candidate_min_yaw_separation = std::max(options_.candidate_min_yaw_separation, 0.0);
  options_.refine_levels = std::max(options_.refine_levels, 0);
  options_.refine_xy_step = std::max(options_.refine_xy_step, 1.0e-3);
  options_.refine_yaw_step = std::max(options_.refine_yaw_step, 1.0e-3);
  options_.scan_stride = std::max(options_.scan_stride, 1);
  options_.free_space_weight = std::max(options_.free_space_weight, 0.0);
  options_.free_space_sample_step = std::max(options_.free_space_sample_step, 1.0e-3);
}

SearchResult LocalizationSearch::run(
  const MapModel & map,
  const LaserScanPoints & endpoints)
{
  SearchResult result;
  if (!map.ready()) {
    result.message = "map is not ready";
    return result;
  }

  result.endpoint_count = endpoints.endpoints.size();
  if (result.endpoint_count < static_cast<std::size_t>(options_.min_endpoint_count)) {
    result.message = "not enough valid lidar endpoints";
    return result;
  }

  auto candidates = coarseSearch(map, endpoints);
  if (candidates.empty()) {
    result.message = "coarse search produced no candidates";
    return result;
  }

  std::vector<Candidate> refined;
  refined.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    refined.push_back(refineCandidate(map, endpoints, candidate));
  }
  std::sort(refined.begin(), refined.end(), [](const auto & lhs, const auto & rhs) {
    return lhs.cost < rhs.cost;
  });

  result.success = true;
  result.message = "localized";
  result.best = refined.front();
  result.candidates = std::move(refined);
  result.covariance = covarianceFromHessian(map, endpoints, result.best.pose);
  return result;
}

double LocalizationSearch::scorePose(
  const MapModel & map,
  const LaserScanPoints& endpoints,
  const Pose2D & pose) const
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  double sum = 0.0;
  double free_space_penalty = 0.0;

  double map_variance = map.resolution() * options_.alpha_m;
  map_variance = map_variance * map_variance;

  for (size_t i = 0; i<endpoints.endpoints.size(); ++i) {
    auto& point = endpoints.endpoints[i];
    auto& variance = endpoints.variances[i];
    double total_variance = variance + map_variance;
    const double wx = pose.x + c * point.x() - s * point.y();
    const double wy = pose.y + s * point.x() + c * point.y();
    const double distance = std::min(
      map.interpolatedDistance(wx, wy, options_.off_map_distance),
      options_.off_map_distance);
    sum += distance * distance / total_variance;

    if (options_.free_space_weight <= 0.0) {
      continue;
    }

    const double range = point.norm();
    if (range <= options_.free_space_sample_step) {
      continue;
    }
    for (double sample_range = options_.free_space_sample_step;
      sample_range < range - options_.free_space_sample_step;
      sample_range += options_.free_space_sample_step)
    {
      const double scale = sample_range / range;
      const double sx = scale * point.x();
      const double sy = scale * point.y();
      const double sample_wx = pose.x + c * sx - s * sy;
      const double sample_wy = pose.y + s * sx + c * sy;
      if (!map.isFreeWorld(sample_wx, sample_wy)) {
        free_space_penalty += options_.off_map_distance * options_.off_map_distance;
      }
    }
  }

  return sum;

  // looks like I gave codex too much rope ...
  // return (sum + options_.free_space_weight * free_space_penalty) /
  //        static_cast<double>(endpoints.size());
}

std::vector<Candidate> LocalizationSearch::coarseSearch(
  const MapModel & map,
  const LaserScanPoints & endpoints)
{

  double map_variance = map.resolution() * options_.alpha_m;
  map_variance = map_variance * map_variance;

  xy_step_cells = std::max<std::uint32_t>(
    1U, static_cast<std::uint32_t>(std::llround(options_.coarse_xy_step / map.resolution())));
  const int yaw_steps = std::max(1, static_cast<int>(std::ceil(kTwoPi / options_.coarse_yaw_step)));
  resolution = map.resolution();

  std::uint32_t num_steps_y = (map.height() + xy_step_cells - 1) / xy_step_cells;
  std::uint32_t num_steps_x = (map.width() + xy_step_cells - 1) / xy_step_cells;

  coarse_score.resize(num_steps_x, num_steps_y);
  coarse_score.fill(std::numeric_limits<double>::infinity());

  std::vector<Candidate> candidates;
  candidates.reserve(static_cast<std::size_t>(options_.top_k));

  auto same_candidate_region = [&](const Candidate & lhs, const Candidate & rhs) {
    if (options_.candidate_min_xy_separation <= 0.0 &&
      options_.candidate_min_yaw_separation <= 0.0)
    {
      return false;
    }
    const double dx = lhs.pose.x - rhs.pose.x;
    const double dy = lhs.pose.y - rhs.pose.y;
    const bool close_xy = options_.candidate_min_xy_separation <= 0.0 ||
      std::hypot(dx, dy) < options_.candidate_min_xy_separation;
    const bool close_yaw = options_.candidate_min_yaw_separation <= 0.0 ||
      std::abs(normalizeYaw(lhs.pose.yaw - rhs.pose.yaw)) < options_.candidate_min_yaw_separation;
    return close_xy && close_yaw;
  };

  auto insert_candidate = [&](const Candidate & candidate) {
    for (auto & existing : candidates) {
      if (same_candidate_region(candidate, existing)) {
        if (candidate.cost < existing.cost) {
          existing = candidate;
        }
        return;
      }
    }

    if (static_cast<int>(candidates.size()) < options_.top_k) {
      candidates.push_back(candidate);
      return;
    }

    auto worst = std::max_element(candidates.begin(), candidates.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.cost < rhs.cost;
    });
    if (worst != candidates.end() && candidate.cost < worst->cost) {
      *worst = candidate;
    }
  };

  for (int yaw_index = 0; yaw_index < yaw_steps; ++yaw_index) {
    const double yaw = normalizeYaw(-kPi + static_cast<double>(yaw_index) * kTwoPi / yaw_steps);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    std::vector<Eigen::Vector2d> rotated;
    rotated.reserve(endpoints.endpoints.size());
    for (const auto & point : endpoints.endpoints) {
      rotated.emplace_back(c * point.x() - s * point.y(), s * point.x() + c * point.y());
    }

    int iy = 0;
    for (std::uint32_t gy = 0; gy < map.height(); gy += xy_step_cells) {
      int ix = 0;
      for (std::uint32_t gx = 0; gx < map.width(); gx += xy_step_cells) {
        double wx = 0.0;
        double wy = 0.0;
        map.gridCenterToWorld(gx, gy, wx, wy);

        double sum = 0.0;
        for (size_t i =0; i < rotated.size(); ++i) {
          auto & point = rotated[i];
          double variance = endpoints.variances[i];
          double total_variance = variance + map_variance;
          const double distance = std::min(
            map.interpolatedDistance(wx + point.x(), wy + point.y(), options_.off_map_distance),
            options_.off_map_distance);
          sum += distance * distance / total_variance;
        }

        //double sum_scaled = sum / static_cast<double>(rotated.size());

        // store the coarse score even if it is verboten
        if (sum < coarse_score(ix, iy)) {
          coarse_score(ix, iy) = sum;
        }
        ix++;

        if (!map.isFreeCell(gx, gy)) {
          continue;
        }

        Candidate candidate{{wx, wy, yaw}, sum};
        insert_candidate(candidate);
      }
      iy++;
    }
  }

  std::sort(candidates.begin(), candidates.end(), [](const auto & lhs, const auto & rhs) {
    return lhs.cost < rhs.cost;
  });
  return candidates;
}

void LocalizationSearch::dumpCoarseScore(const std::string& filename)
{
    std::ofstream file(filename);
    if (!file) return;

    double step = xy_step_cells * resolution;
    double x0 = 0; 
    double y0 = 0;

    for (int c = 0; c < coarse_score.cols(); ++c) {
        for (int r = 0; r < coarse_score.rows(); ++r) {
            // Calculate world coordinates for this specific pixel
            double world_x = x0 + r * step;
            double world_y = y0 + c * step;
            
            file << world_x << " " << world_y << " " << coarse_score(r, c) << "\n";
        }
        // Gnuplot likes a blank line between columns for grid data
        file << "\n"; 
    }
}

Candidate LocalizationSearch::refineCandidate(
  const MapModel & map,
  const LaserScanPoints & endpoints,
  const Candidate & initial) const
{
  Candidate best = initial;
  double xy_step = options_.refine_xy_step;
  double yaw_step = options_.refine_yaw_step;

  for (int level = 0; level < options_.refine_levels; ++level) {
    bool improved = true;
    while (improved) {
      improved = false;
      Candidate level_best = best;
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dyaw = -1; dyaw <= 1; ++dyaw) {
            if (dx == 0 && dy == 0 && dyaw == 0) {
              continue;
            }
            Candidate candidate;
            candidate.pose.x = best.pose.x + static_cast<double>(dx) * xy_step;
            candidate.pose.y = best.pose.y + static_cast<double>(dy) * xy_step;
            candidate.pose.yaw = normalizeYaw(best.pose.yaw + static_cast<double>(dyaw) * yaw_step);
            candidate.cost = scorePose(map, endpoints, candidate.pose);
            if (candidate.cost < level_best.cost) {
              level_best = candidate;
            }
          }
        }
      }
      if (level_best.cost + 1.0e-12 < best.cost) {
        best = level_best;
        improved = true;
      }
    }
    xy_step *= 0.5;
    yaw_step *= 0.5;
  }

  return best;
}

Eigen::Matrix3d LocalizationSearch::covarianceFromHessian(
  const MapModel & map,
  const LaserScanPoints & endpoints,
  const Pose2D & pose) const
{
  const Eigen::Vector3d step(
    std::max(options_.covariance_step_xy, 1.0e-4),
    std::max(options_.covariance_step_xy, 1.0e-4),
    std::max(options_.covariance_step_yaw, 1.0e-4));
  Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
  const double f0 = scorePose(map, endpoints, pose);

  for (int i = 0; i < 3; ++i) {
    Pose2D plus = pose;
    Pose2D minus = pose;
    if (i == 0) {
      plus.x += step(i);
      minus.x -= step(i);
    } else if (i == 1) {
      plus.y += step(i);
      minus.y -= step(i);
    } else {
      plus.yaw = normalizeYaw(plus.yaw + step(i));
      minus.yaw = normalizeYaw(minus.yaw - step(i));
    }
    const double fp = scorePose(map, endpoints, plus);
    const double fm = scorePose(map, endpoints, minus);
    hessian(i, i) = (fp - 2.0 * f0 + fm) / (step(i) * step(i));

    for (int j = i + 1; j < 3; ++j) {
      Pose2D pp = pose;
      Pose2D pm = pose;
      Pose2D mp = pose;
      Pose2D mm = pose;
      auto add_step = [&](Pose2D & p, int index, double delta) {
        if (index == 0) {
          p.x += delta;
        } else if (index == 1) {
          p.y += delta;
        } else {
          p.yaw = normalizeYaw(p.yaw + delta);
        }
      };
      add_step(pp, i, step(i));
      add_step(pp, j, step(j));
      add_step(pm, i, step(i));
      add_step(pm, j, -step(j));
      add_step(mp, i, -step(i));
      add_step(mp, j, step(j));
      add_step(mm, i, -step(i));
      add_step(mm, j, -step(j));
      const double value = (scorePose(map, endpoints, pp) - scorePose(map, endpoints, pm) -
        scorePose(map, endpoints, mp) + scorePose(map, endpoints, mm)) /
        (4.0 * step(i) * step(j));
      hessian(i, j) = value;
      hessian(j, i) = value;
    }
  }

  hessian += options_.covariance_regularization * Eigen::Matrix3d::Identity();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(hessian);
  if (solver.info() != Eigen::Success) {
    return Eigen::Matrix3d::Identity() * 1.0e3;
  }

  Eigen::Vector3d eigenvalues = solver.eigenvalues();
  for (int i = 0; i < 3; ++i) {
    eigenvalues(i) = 1.0 / std::max(eigenvalues(i), options_.covariance_regularization);
  }
  return options_.covariance_scale *
         solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
}

}  // namespace map_solver
