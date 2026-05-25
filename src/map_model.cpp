#include "map_solver/map_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/imgproc.hpp>

namespace map_solver
{

bool MapModel::update(const nav_msgs::msg::OccupancyGrid & map, const MapBuildOptions & options)
{
  if (map.info.width == 0 || map.info.height == 0 || map.info.resolution <= 0.0F) {
    ready_ = false;
    return false;
  }

  const auto expected_size = static_cast<std::size_t>(map.info.width) *
    static_cast<std::size_t>(map.info.height);
  if (map.data.size() != expected_size) {
    ready_ = false;
    return false;
  }

  frame_id_ = map.header.frame_id.empty() ? "map" : map.header.frame_id;
  resolution_ = static_cast<double>(map.info.resolution);
  origin_x_ = map.info.origin.position.x;
  origin_y_ = map.info.origin.position.y;
  origin_yaw_ = yawFromQuaternion(map.info.origin.orientation);
  origin_cos_ = std::cos(origin_yaw_);
  origin_sin_ = std::sin(origin_yaw_);
  width_ = map.info.width;
  height_ = map.info.height;
  edt_padding_cells_ = static_cast<std::uint32_t>(std::max(
    0.0, std::ceil(options.map_padding_xy / resolution_)));
  edt_width_ = width_ + 2U * edt_padding_cells_;
  edt_height_ = height_ + 2U * edt_padding_cells_;
  occupancy_ = map.data;
  occupied_threshold_ = options.occupied_threshold;
  unknown_is_occupied_ = options.unknown_is_occupied;

  cv::Mat obstacle_image(static_cast<int>(height_), static_cast<int>(width_), CV_8UC1);
  for (std::uint32_t y = 0; y < height_; ++y) {
    auto * row = obstacle_image.ptr<std::uint8_t>(static_cast<int>(y));
    for (std::uint32_t x = 0; x < width_; ++x) {
      const auto value = occupancy_[static_cast<std::size_t>(y) * width_ + x];
      const bool occupied = value >= occupied_threshold_ || (value < 0 && unknown_is_occupied_);
      row[x] = occupied ? 0U : 255U;
    }
  }

  cv::Mat padded_obstacle_image;
  if (edt_padding_cells_ > 0U) {
    cv::copyMakeBorder(
      obstacle_image,
      padded_obstacle_image,
      static_cast<int>(edt_padding_cells_),
      static_cast<int>(edt_padding_cells_),
      static_cast<int>(edt_padding_cells_),
      static_cast<int>(edt_padding_cells_),
      cv::BORDER_CONSTANT,
      cv::Scalar(255));
  } else {
    padded_obstacle_image = obstacle_image;
  }

  cv::Mat edt_cells;
  cv::distanceTransform(padded_obstacle_image, edt_cells, cv::DIST_L2, cv::DIST_MASK_PRECISE);
  edt_cells.convertTo(edt_meters_, CV_32FC1, resolution_);
  ready_ = true;
  return true;
}

bool MapModel::isFreeCell(std::uint32_t x, std::uint32_t y) const
{
  if (x >= width_ || y >= height_ || occupancy_.empty()) {
    return false;
  }
  const auto value = occupancy_[static_cast<std::size_t>(y) * width_ + x];
  if (value < 0) {
    return !unknown_is_occupied_;
  }
  return value < occupied_threshold_;
}

bool MapModel::isFreeWorld(double wx, double wy) const
{
  double gx = 0.0;
  double gy = 0.0;
  if (!worldToGrid(wx, wy, gx, gy)) {
    return false;
  }

  const auto cell_x = static_cast<std::uint32_t>(std::llround(gx));
  const auto cell_y = static_cast<std::uint32_t>(std::llround(gy));
  return isFreeCell(cell_x, cell_y);
}

bool MapModel::worldToGridUnbounded(double wx, double wy, double & gx, double & gy) const
{
  const double dx = wx - origin_x_;
  const double dy = wy - origin_y_;
  const double local_x = origin_cos_ * dx + origin_sin_ * dy;
  const double local_y = -origin_sin_ * dx + origin_cos_ * dy;
  gx = local_x / resolution_ - 0.5;
  gy = local_y / resolution_ - 0.5;
  return true;
}

bool MapModel::worldToGrid(double wx, double wy, double & gx, double & gy) const
{
  worldToGridUnbounded(wx, wy, gx, gy);
  return gx >= 0.0 && gy >= 0.0 && gx <= static_cast<double>(width_ - 1) &&
         gy <= static_cast<double>(height_ - 1);
}

void MapModel::gridCenterToWorld(std::uint32_t gx, std::uint32_t gy, double & wx, double & wy) const
{
  const double local_x = (static_cast<double>(gx) + 0.5) * resolution_;
  const double local_y = (static_cast<double>(gy) + 0.5) * resolution_;
  wx = origin_x_ + origin_cos_ * local_x - origin_sin_ * local_y;
  wy = origin_y_ + origin_sin_ * local_x + origin_cos_ * local_y;
}

double MapModel::interpolatedDistance(double wx, double wy, double off_map_distance) const
{
  if (!ready_ || edt_meters_.empty()) {
    return off_map_distance;
  }

  double gx = 0.0;
  double gy = 0.0;
  worldToGridUnbounded(wx, wy, gx, gy);

  const double padded_gx = gx + static_cast<double>(edt_padding_cells_);
  const double padded_gy = gy + static_cast<double>(edt_padding_cells_);
  if (padded_gx < 0.0 || padded_gy < 0.0 ||
    padded_gx > static_cast<double>(edt_width_ - 1) ||
    padded_gy > static_cast<double>(edt_height_ - 1))
  {
    return off_map_distance;
  }

  const int x0 = static_cast<int>(std::floor(padded_gx));
  const int y0 = static_cast<int>(std::floor(padded_gy));
  const int x1 = std::min(x0 + 1, static_cast<int>(edt_width_ - 1));
  const int y1 = std::min(y0 + 1, static_cast<int>(edt_height_ - 1));
  const double tx = std::clamp(padded_gx - static_cast<double>(x0), 0.0, 1.0);
  const double ty = std::clamp(padded_gy - static_cast<double>(y0), 0.0, 1.0);

  const float d00 = edt_meters_.at<float>(y0, x0);
  const float d10 = edt_meters_.at<float>(y0, x1);
  const float d01 = edt_meters_.at<float>(y1, x0);
  const float d11 = edt_meters_.at<float>(y1, x1);

  const double d0 = (1.0 - tx) * d00 + tx * d10;
  const double d1 = (1.0 - tx) * d01 + tx * d11;
  return (1.0 - ty) * d0 + ty * d1;
}

}  // namespace map_solver
