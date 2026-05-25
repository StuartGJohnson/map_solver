#ifndef MAP_SOLVER__MAP_MODEL_HPP_
#define MAP_SOLVER__MAP_MODEL_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core.hpp>

namespace map_solver
{

inline double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q_msg)
{
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

struct MapBuildOptions
{
  int occupied_threshold{50};
  bool unknown_is_occupied{false};
  double map_padding_xy{1.0};
};

class MapModel
{
public:
  bool update(const nav_msgs::msg::OccupancyGrid & map, const MapBuildOptions & options);

  bool ready() const {return ready_;}
  const std::string & frame_id() const {return frame_id_;}
  double resolution() const {return resolution_;}
  double origin_x() const {return origin_x_;}
  double origin_y() const {return origin_y_;}
  double origin_yaw() const {return origin_yaw_;}
  std::uint32_t width() const {return width_;}
  std::uint32_t height() const {return height_;}

  bool isFreeCell(std::uint32_t x, std::uint32_t y) const;
  bool isFreeWorld(double wx, double wy) const;
  bool worldToGrid(double wx, double wy, double & gx, double & gy) const;
  void gridCenterToWorld(std::uint32_t gx, std::uint32_t gy, double & wx, double & wy) const;
  double interpolatedDistance(double wx, double wy, double off_map_distance) const;

private:
  bool worldToGridUnbounded(double wx, double wy, double & gx, double & gy) const;

  bool ready_{false};
  std::string frame_id_{"map"};
  double resolution_{0.05};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_yaw_{0.0};
  double origin_cos_{1.0};
  double origin_sin_{0.0};
  std::uint32_t width_{0};
  std::uint32_t height_{0};
  std::uint32_t edt_width_{0};
  std::uint32_t edt_height_{0};
  std::uint32_t edt_padding_cells_{0};
  std::vector<std::int8_t> occupancy_;
  cv::Mat edt_meters_;
  int occupied_threshold_{50};
  bool unknown_is_occupied_{false};
};

}  // namespace map_solver

#endif  // MAP_SOLVER__MAP_MODEL_HPP_
