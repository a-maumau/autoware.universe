// Copyright 2022 TIER IV, Inc.
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

#include "autoware/detected_object_validation/utils/utils.hpp"

#include <autoware_perception_msgs/msg/object_classification.hpp>

namespace autoware::detected_object_validation
{
namespace utils
{
using Label = autoware_perception_msgs::msg::ObjectClassification;
using TriangleMesh = std::vector<std::array<Eigen::Vector3d, 3>>;

bool FilterTargetLabel::isTarget(const uint8_t label) const
{
  return (label == Label::UNKNOWN && UNKNOWN) || (label == Label::CAR && CAR) ||
         (label == Label::TRUCK && TRUCK) || (label == Label::BUS && BUS) ||
         (label == Label::TRAILER && TRAILER) || (label == Label::MOTORCYCLE && MOTORCYCLE) ||
         (label == Label::BICYCLE && BICYCLE) || (label == Label::PEDESTRIAN && PEDESTRIAN);
}

TriangleMesh createTriangleMeshFromLanelet(const lanelet::ConstLanelet & lanelet) {
  TriangleMesh triangles;

  const lanelet::ConstLineString3d & left = lanelet.leftBound();
  const lanelet::ConstLineString3d & right = lanelet.rightBound();

  // bounds must have same number of points
  // if not the same, only check the first shared points
  size_t n = std::min(left.size(), right.size());
  if (n < 2) continue;

  // take 2 points from each side and create 2 triangles
  for (size_t i = 0; i < n - 1; ++i) {
    Eigen::Vector3d a_l(left[i].x(), left[i].y(), left[i].z());
    Eigen::Vector3d b_l(left[i + 1].x(), left[i + 1].y(), left[i + 1].z());
    Eigen::Vector3d a_r(right[i].x(), right[i].y(), right[i].z());
    Eigen::Vector3d b_r(right[i + 1].x(), right[i + 1].y(), right[i + 1].z());

    //
    // b_l .--. b_r
    //     |\ |
    //     | \|
    //     .--.
    // a_l      a_r
    //
    triangles.push_back({a_l, b_l, a_r});
    triangles.push_back({b_l, b_r, a_r});
  }

  return triangles;
}

// Ensure the normal is pointing upward (Z+)
Point3D computeUpwardNormal(const Triangle& tri) {
    Point3D v1 = tri[1] - tri[0];
    Point3D v2 = tri[2] - tri[0];
    Point3D normal = v1.cross(v2);
    if (normal.z() < 0) {
        normal = -normal;
    }
    return normalize(normal);
}

// Barycentric check for 3D triangle
bool pointInTriangle(const Point3D& p, const Triangle& tri) {
    Point3D v0 = tri[2] - tri[0];
    Point3D v1 = tri[1] - tri[0];
    Point3D v2 = p - tri[0];

    double dot00 = v0.dot(v0);
    double dot01 = v0.dot(v1);
    double dot02 = v0.dot(v2);
    double dot11 = v1.dot(v1);
    double dot12 = v1.dot(v2);

    double denom = dot00 * dot11 - dot01 * dot01;
    if (std::abs(denom) < 1e-8) return false;

    double u = (dot11 * dot02 - dot01 * dot12) / denom;
    double v = (dot00 * dot12 - dot01 * dot02) / denom;
    return (u >= 0) && (v >= 0) && (u + v <= 1);
}


}  // namespace utils
}  // namespace autoware::detected_object_validation
