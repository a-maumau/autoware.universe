// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__CAMERA_PROJECTION_HPP_
#define AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__CAMERA_PROJECTION_HPP_

#define EIGEN_MPL2_ONLY

#include <Eigen/Core>

#include <autoware/universe_utils/system/lru_cache.hpp>

#include <sensor_msgs/msg/camera_info.hpp>

#include <opencv2/core/core.hpp>

#include <image_geometry/pinhole_camera_model.h>

#include <map>
#include <memory>

namespace autoware::image_projection_based_fusion
{

class CameraProjection
{
public:
  explicit CameraProjection(
    const sensor_msgs::msg::CameraInfo & camera_info,
    const float grid_size,
    const bool unrectify,
    const bool use_approximation
  );
  CameraProjection(): grid_size_(1.0), unrectify_(false) {};
  bool calcRawImageProjectedPoint(const cv::Point3d & point3d, Eigen::Vector2d & projected_point);
  sensor_msgs::msg::CameraInfo getCameraInfo();
  bool isOutsideHorizontalView(float px, float pz);
  bool isOutsideVerticalView(float py, float pz);

protected:
  void initializeCache();

  sensor_msgs::msg::CameraInfo camera_info_;
  uint32_t image_height_, image_width_;
  double tan_h_x_, tan_h_y_;

  uint32_t cache_size_;
  float grid_size_;
  float half_grid_size_;
  float inv_grid_size_;
  bool unrectify_;
  bool use_approximation_;

  std::shared_ptr<autoware::universe_utils::LRUCache<float, Eigen::Vector2d>> projection_cache_;
  image_geometry::PinholeCameraModel camera_model_;
};

}  // namespace autoware::image_projection_based_fusion

#endif  // AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__ROI_CLUSTER_FUSION__NODE_HPP_
