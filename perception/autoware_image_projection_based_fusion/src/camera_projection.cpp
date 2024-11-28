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

#include "autoware/image_projection_based_fusion/camera_projection.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

namespace autoware::image_projection_based_fusion
{
CameraProjection::CameraProjection(
  const sensor_msgs::msg::CameraInfo & camera_info, const float grid_size, const bool unrectify, const bool use_approximation=false
): camera_info_(camera_info), grid_size_(grid_size), unrectify_(unrectify), use_approximation_(use_approximation){
  // prepare camera model
  camera_model_.fromCameraInfo(camera_info);

  // projection cache settings
  image_height_ = camera_info_.height;
  image_width_ = camera_info_.width;
  half_grid_size_ = grid_size_ / 2.0; // for shifting to the grid center
  inv_grid_size_ = 1/grid_size_;
  cache_size_ = std::round(image_height_/grid_size_)*std::round(image_width_/grid_size_);

  // for checking the views
  // cx/fx
  tan_h_x_ = camera_info.k.at(2)/camera_info.k.at(0);
  // cy/fy
  tan_h_y_ = camera_info.k.at(5)/camera_info.k.at(4);

  if (use_approximation_) {
    // if we need to consider the result of rectification, we need to adjust the cache size
    projection_cache_ =
      std::make_shared<autoware::universe_utils::LRUCache<float, Eigen::Vector2d>>(cache_size_);
    initializeCache();
  }
}

void CameraProjection::initializeCache(){
  // sample all pixel values till the camera height, width
  // TODO:
  //   consider the rectfied image coordinate
  //
  //      grid_size
  //      /
  //     v
  //   |---|          w
  //  0----------------->
  // 0 | . | . | . |
  //   |___|___|___|
  //   | . | . | . |
  //   | ^
  // h | |
  //   v grid center
  //
  // each pixel will be rounded in this grid center

  for (float y = 0.0; y < image_height_; y += grid_size_) {
    for (float x = 0.0; x < image_width_; x += grid_size_) {
  //for (int y = 0; y < image_height_; y++) {
  //  for (int x = 0; x < image_width_; x++) {
      // round to a near grid center
      //const int qx = static_cast<uint32_t>((x-half_grid_size_)/grid_size_)*grid_size_+half_grid_size_;
      //const int qy = static_cast<uint32_t>((y-half_grid_size_)/grid_size_)*grid_size_+half_grid_size_;
      //const int cache_key = static_cast<uint32_t>(qx+qy*image_width_);
      const float qx = std::round(x*inv_grid_size_)*grid_size_+half_grid_size_;
      const float qy = std::round(y*inv_grid_size_)*grid_size_+half_grid_size_;
      const float cache_key = (qx+qy*image_width_);
      //std::cout << cache_key << std::endl;

      // precompute projected point
      cv::Point2d raw_image_point = camera_model_.unrectifyPoint(cv::Point2d(qx, qy));
      Eigen::Vector2d projection_point(raw_image_point.x, raw_image_point.y);

      projection_cache_->put(cache_key, projection_point);
    }
  }
}

/**
 * @brief calculate a projection of 3D point to image plane 2D point. 
 * @return return the projected points that is on the image plane or not.
 */
bool CameraProjection::calcRawImageProjectedPoint(
  const cv::Point3d & point3d, Eigen::Vector2d & projected_point
){
  const cv::Point2d rectified_image_point = camera_model_.project3dToPixel(point3d);

  if (!unrectify_) {
    if (rectified_image_point.x < 0.0 || rectified_image_point.x >= image_width_ || rectified_image_point.y < 0.0 || rectified_image_point.y >= image_height_){
      return false;
    } else {
      projected_point << rectified_image_point.x, rectified_image_point.y;
      return true;
    }
  }

  if(use_approximation_) {
    // round to a near grid center
    const float qx = std::round(rectified_image_point.x*inv_grid_size_)*grid_size_+half_grid_size_;
    if (qx < 0.0 || qx >= image_width_) {
      return false;
    }

    const float qy = std::round(rectified_image_point.y*inv_grid_size_)*grid_size_+half_grid_size_;
    if (qy < 0.0 || qy >= image_height_) {
      return false;
    }

    const float cache_key = (qx+qy*image_width_);
    // if rounded position is already in the cache, then use it as an approximation
    if (projection_cache_->contains(cache_key)) {
      projected_point = *projection_cache_->get(cache_key);
      return true;
    } else {
      // if cache misses, consider as a invalid position
      return false;
    }
  } else {
    const cv::Point2d raw_image_point = camera_model_.unrectifyPoint(rectified_image_point);
    if (rectified_image_point.x < 0.0 || rectified_image_point.x >= image_width_ || rectified_image_point.y < 0.0 || rectified_image_point.y >= image_height_){
      return false;
    } else {
      projected_point << raw_image_point.x, raw_image_point.y;
      return true;
    }
  }
}

sensor_msgs::msg::CameraInfo CameraProjection::getCameraInfo(){
  return camera_info_;
}

bool CameraProjection::isOutsideHorizontalView(float px, float pz){
  // assuming the points' origin is centered on the camera
  return pz <= 0.0 || px > tan_h_x_ * pz || px < -tan_h_x_ * pz;
}

bool CameraProjection::isOutsideVerticalView(float py, float pz){
  // assuming the points' origin is centered on the camera
  return pz <= 0.0 || py > tan_h_y_ * pz || py < -tan_h_y_ * pz;
}

}  // namespace autoware::image_projection_based_fusion
