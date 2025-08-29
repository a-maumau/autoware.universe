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

#include "lanelet_filter_base.hpp"

#include "autoware/object_recognition_utils/object_recognition_utils.hpp"
#include "autoware_lanelet2_extension/utility/message_conversion.hpp"
#include "autoware_lanelet2_extension/utility/query.hpp"
#include "autoware_utils/geometry/geometry.hpp"

#include <Eigen/Core>

#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::detected_object_validation
{
namespace lanelet_filter
{
using Polygon = bg::model::polygon<Point2d>;
using LaneletEntry = std::pair<Box, lanelet::Lanelet>;
using LaneletEntryMap = std::unordered_map<lanelet::Id, LaneletEntry>;

// using TriangleMesh = std::vector<std::array<Eigen::Vector3d, 3>>;
using autoware_utils::ScopedTimeTrack;

visualization_msgs::msg::Marker create_point_marker(std::vector<Eigen::Vector3d> query_points)
{
  auto marker = visualization_msgs::msg::Marker();
  marker.type = visualization_msgs::msg::Marker::POINTS;

  marker.header.frame_id = "map";  // or your reference frame
  marker.ns = "my_points";
  marker.id = 244;
  marker.lifetime = rclcpp::Duration::from_seconds(0.5) marker.type =
    visualization_msgs::msg::Marker::POINTS;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.25;  // width
  marker.scale.y = 0.25;  // height
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  for (const auto & qp : query_points) {
    geometry_msgs::msg::Point pt;
    pt.x = qp.x();
    pt.y = qp.y();
    pt.z = qp.z();
    marker.points.push_back(pt);
  }

  return marker;
}

visualization_msgs::msg::Marker create_triangle_marker(
  std::vector<std::array<Eigen::Vector3d, 3>> nearest_triangles)
{
  auto tri_marker = visualization_msgs::msg::Marker();
  tri_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;

  tri_marker.header.frame_id = "map";
  tri_marker.ns = "my_triangles";
  tri_marker.id = 245;
  tri_marker.lifetime = rclcpp::Duration::from_seconds(0.5) tri_marker.type =
    visualization_msgs::msg::Marker::TRIANGLE_LIST;
  tri_marker.action = visualization_msgs::msg::Marker::ADD;
  tri_marker.scale.x = tri_marker.scale.y = tri_marker.scale.z = 1.0;
  tri_marker.color.r = 0.0;
  tri_marker.color.g = 1.0;
  tri_marker.color.b = 0.0;
  tri_marker.color.a = 0.7;

  // Each group of 3 points forms a triangle
  geometry_msgs::msg::Point p1, p2, p3;
  for (const auto & tri : nearest_triangles) {
    for (int i = 0; i < 3; ++i) {
      geometry_msgs::msg::Point pt;
      pt.x = tri[i].x();
      pt.y = tri[i].y();
      pt.z = tri[i].z() + 0.1;
      tri_marker.points.push_back(pt);
    }
  }

  return tri_marker;
}

visualization_msgs::msg::Marker create_relation_line_marker(
  std::vector<Eigen::Vector3d> query_points,
  std::vector<std::array<Eigen::Vector3d, 3>> nearest_triangles)
{
  auto line_marker = visualization_msgs::msg::Marker();
  line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;

  line_marker.header.frame_id = "map";
  line_marker.ns = "point_to_vertex";
  line_marker.id = 246;
  line_marker.lifetime = rclcpp::Duration::from_seconds(0.5) line_marker.action =
    visualization_msgs::msg::Marker::ADD;
  line_marker.scale.x = 0.05;  // Line width
  line_marker.color.r = 0.8;
  line_marker.color.g = 0.8;
  line_marker.color.b = 0.0;
  line_marker.color.a = 0.7;

  // Each group of 3 points forms a triangle
  geometry_msgs::msg::Point p1, p2, p3;
  for (size_t i = 0; i < query_points.size(); i++) {
    auto qp = query_points[i];
    geometry_msgs::msg::Point pt;
    pt.x = qp.x();
    pt.y = qp.y();
    pt.z = qp.z();

    auto tri = nearest_triangles[i];
    geometry_msgs::msg::Point v1, v2, v3;
    v1.x = tri[0].x();
    v1.y = tri[0].y();
    v1.z = tri[0].z() + 0.1;

    v2.x = tri[1].x();
    v2.y = tri[1].y();
    v2.z = tri[1].z() + 0.1;

    v3.x = tri[2].x();
    v3.y = tri[2].y();
    v3.z = tri[2].z() + 0.1;

    // create line from query point to vertices
    line_marker.points.push_back(pt);
    line_marker.points.push_back(v1);

    line_marker.points.push_back(pt);
    line_marker.points.push_back(v2);

    line_marker.points.push_back(pt);
    line_marker.points.push_back(v3);
  }

  return line_marker;
}

template <typename ObjsMsgType, typename ObjMsgType>
ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::ObjectLaneletFilterBase(
  const std::string & node_name, const rclcpp::NodeOptions & node_options)
: Node(node_name, node_options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;

  // Set parameters
  filter_target_.UNKNOWN = declare_parameter<bool>("filter_target_label.UNKNOWN");
  filter_target_.CAR = declare_parameter<bool>("filter_target_label.CAR");
  filter_target_.TRUCK = declare_parameter<bool>("filter_target_label.TRUCK");
  filter_target_.BUS = declare_parameter<bool>("filter_target_label.BUS");
  filter_target_.TRAILER = declare_parameter<bool>("filter_target_label.TRAILER");
  filter_target_.MOTORCYCLE = declare_parameter<bool>("filter_target_label.MOTORCYCLE");
  filter_target_.BICYCLE = declare_parameter<bool>("filter_target_label.BICYCLE");
  filter_target_.PEDESTRIAN = declare_parameter<bool>("filter_target_label.PEDESTRIAN");

  // Set filter settings
  filter_settings_.lanelet_xy_overlap_filter =
    declare_parameter<bool>("filter_settings.lanelet_xy_overlap_filter.enabled");

  filter_settings_.lanelet_direction_filter =
    declare_parameter<bool>("filter_settings.lanelet_direction_filter.enabled");
  filter_settings_.lanelet_direction_filter_velocity_yaw_threshold =
    declare_parameter<double>("filter_settings.lanelet_direction_filter.velocity_yaw_threshold");
  filter_settings_.lanelet_direction_filter_object_speed_threshold =
    declare_parameter<double>("filter_settings.lanelet_direction_filter.object_speed_threshold");

  filter_settings_.lanelet_object_elevation_filter =
    declare_parameter<bool>("filter_settings.lanelet_object_elevation_filter.enabled");
  filter_settings_.max_elevation_threshold = declare_parameter<double>(
    "filter_settings.lanelet_object_elevation_filter.max_elevation_threshold");
  filter_settings_.min_elevation_threshold = declare_parameter<double>(
    "filter_settings.lanelet_object_elevation_filter.min_elevation_threshold");
  const double abs_max_elevation =
    std::max(filter_settings_.max_elevation_threshold, filter_settings_.min_elevation_threshold);
  elevation_min_z_dist_sq_ = abs_max_elevation * abs_max_elevation;

  filter_settings_.lanelet_extra_margin =
    declare_parameter<double>("filter_settings.lanelet_extra_margin");
  filter_settings_.debug = declare_parameter<bool>("filter_settings.debug");

  if (filter_settings_.min_elevation_threshold > filter_settings_.max_elevation_threshold) {
    RCLCPP_WARN(
      this->get_logger(),
      "parameters of object_elevation_filter do not satisfy the relation: "
      "min_elevation_threshold (%f) <= max_elevation_threshold (%f)",
      filter_settings_.min_elevation_threshold, filter_settings_.max_elevation_threshold);
  }

  // Set publisher/subscriber
  map_sub_ = this->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&ObjectLaneletFilterBase::mapCallback, this, _1));
  object_sub_ = this->create_subscription<ObjsMsgType>(
    "input/object", rclcpp::QoS{1}, std::bind(&ObjectLaneletFilterBase::objectCallback, this, _1));
  object_pub_ = this->create_publisher<ObjsMsgType>("output/object", rclcpp::QoS{1});

  debug_publisher_ =
    std::make_unique<autoware_utils::DebugPublisher>(this, "object_lanelet_filter");
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  if (filter_settings_.debug) {
    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/debug/marker", rclcpp::QoS{1});
  }

  detailed_processing_time_publisher_ =
    this->create_publisher<autoware_utils::ProcessingTimeDetail>(
      "~/debug/processing_time_detail_ms", 1);
  auto time_keeper = autoware_utils::TimeKeeper(detailed_processing_time_publisher_);
  time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(time_keeper);

  point_marker_pub_ =
    create_publisher<visualization_msgs::msg::Marker>("~/debug/query_marker", rclcpp::QoS{1});
  triangle_merker_pub_ =
    create_publisher<visualization_msgs::msg::Marker>("~/debug/triangle_marker", rclcpp::QoS{1});
  query_to_vertex_merker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
    "~/debug/query_to_vertex_marker", rclcpp::QoS{1});
}

template <typename ObjsMsgType, typename ObjMsgType>
ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::~ObjectLaneletFilterBase()
{
  // join all worker threads to ensure clean shutdown
  for (auto & t : worker_threads_) {
    if (t.joinable()) t.join();
  }
}

bool isInPolygon(
  const geometry_msgs::msg::Pose & current_pose, const lanelet::BasicPolygon2d & polygon,
  const double & radius)
{
  constexpr double eps = 1.0e-9;
  const lanelet::BasicPoint2d p(current_pose.position.x, current_pose.position.y);
  return boost::geometry::distance(p, polygon) < radius + eps;
}

bool isInPolygon(
  const double & x, const double & y, const lanelet::BasicPolygon2d & polygon,
  const double & radius)
{
  constexpr double eps = 1.0e-9;
  const lanelet::BasicPoint2d p(x, y);
  return boost::geometry::distance(p, polygon) < radius + eps;
}

double distFromClosestPolygon(
  const double & x, const double & y, const lanelet::BasicPolygon2d & polygon)
{
  const lanelet::BasicPoint2d p(x, y);
  return boost::geometry::distance(p, polygon);
}

LinearRing2d expandPolygon(const LinearRing2d & polygon, double distance)
{
  autoware_utils::MultiPolygon2d multi_polygon;
  bg::strategy::buffer::distance_symmetric<double> distance_strategy(distance);
  bg::strategy::buffer::join_miter join_strategy;
  bg::strategy::buffer::end_flat end_strategy;
  bg::strategy::buffer::point_circle circle_strategy;
  bg::strategy::buffer::side_straight side_strategy;

  bg::buffer(
    polygon, multi_polygon, distance_strategy, side_strategy, join_strategy, end_strategy,
    circle_strategy);

  if (multi_polygon.empty()) {
    return polygon;
  }

  return multi_polygon.front().outer();
}

TriangleMesh createTriangleMeshFromLanelet(const lanelet::ConstLanelet & lanelet)
{
  TriangleMesh mesh;

  const lanelet::ConstLineString3d & left = lanelet.leftBound();
  const lanelet::ConstLineString3d & right = lanelet.rightBound();

  // bounds should have same number of points
  // if not the same, first check the first shared points then we will check it from the tail
  // since we don't have the original set of 3 points,
  // we will approximate the 3d mesh with this way
  size_t n_left = left.size();
  size_t n_right = right.size();
  const size_t n_shared = std::min(n_left, n_right);
  if (n_shared < 2) return mesh;

  // take 2 points from each side and create 2 triangles
  for (size_t i = 0; i < n_shared - 1; ++i) {
    const Eigen::Vector3d a_l(left[i].x(), left[i].y(), left[i].z());
    const Eigen::Vector3d b_l(left[i + 1].x(), left[i + 1].y(), left[i + 1].z());
    const Eigen::Vector3d a_r(right[i].x(), right[i].y(), right[i].z());
    const Eigen::Vector3d b_r(right[i + 1].x(), right[i + 1].y(), right[i + 1].z());

    //
    // b_l .--. b_r
    //     |\ |
    //     | \|
    //     .--.
    // a_l      a_r
    //
    // mesh.push_back({a_l, a_r, b_l});
    // mesh.push_back({b_r, b_l, a_r});
    // Triangle 1
    mesh.push_back(Triangle{
      {Point2d(a_l.x(), a_l.y()), Point2d(a_r.x(), a_r.y()), Point2d(b_l.x(), b_l.y())},
      {a_l, a_r, b_l},
      mesh.size()});
    // Triangle 2
    mesh.push_back(Triangle{
      {Point2d(b_r.x(), b_r.y()), Point2d(b_l.x(), b_l.y()), Point2d(a_r.x(), a_r.y())},
      {b_r, b_l, a_r},
      mesh.size()});
  }

  // triangulation of the remaining unmatched parts from the tail
  if (n_left > n_right) {
    size_t i = n_left - 1;
    size_t j = n_right - 1;
    const size_t n_extra = n_left - n_right;

    for (size_t k = 0; k < n_extra; ++k) {
      // we need at least 2 points from each side
      if (i < 1 || j < 1) break;

      const Eigen::Vector3d a_l(left[i - 1].x(), left[i - 1].y(), left[i - 1].z());
      const Eigen::Vector3d b_l(left[i].x(), left[i].y(), left[i].z());
      const Eigen::Vector3d a_r(right[j - 1].x(), right[j - 1].y(), right[j - 1].z());
      const Eigen::Vector3d b_r(right[j].x(), right[j].y(), right[j].z());

      // mesh.push_back({b_l, a_l, b_r});
      // mesh.push_back({a_l, a_r, b_r});
      //  Triangle 1
      mesh.push_back(Triangle{
        {Point2d(a_l.x(), a_l.y()), Point2d(a_r.x(), a_r.y()), Point2d(b_l.x(), b_l.y())},
        {a_l, a_r, b_l},
        mesh.size()});
      // Triangle 2
      mesh.push_back(Triangle{
        {Point2d(b_r.x(), b_r.y()), Point2d(b_l.x(), b_l.y()), Point2d(a_r.x(), a_r.y())},
        {b_r, b_l, a_r},
        mesh.size()});

      --i;
      --j;
    }
  } else if (n_right > n_left) {
    size_t i = n_left - 1;
    size_t j = n_right - 1;
    const size_t n_extra = n_right - n_left;

    for (size_t k = 0; k < n_extra; ++k) {
      if (i < 1 || j < 1) break;

      const Eigen::Vector3d a_l(left[i - 1].x(), left[i - 1].y(), left[i - 1].z());
      const Eigen::Vector3d b_l(left[i].x(), left[i].y(), left[i].z());
      const Eigen::Vector3d a_r(right[j - 1].x(), right[j - 1].y(), right[j - 1].z());
      const Eigen::Vector3d b_r(right[j].x(), right[j].y(), right[j].z());

      // mesh.push_back({b_l, a_l, b_r});
      // mesh.push_back({a_l, a_r, b_r});
      //  Triangle 1
      mesh.push_back(Triangle{
        {Point2d(a_l.x(), a_l.y()), Point2d(a_r.x(), a_r.y()), Point2d(b_l.x(), b_l.y())},
        {a_l, a_r, b_l},
        mesh.size()});
      // Triangle 2
      mesh.push_back(Triangle{
        {Point2d(b_r.x(), b_r.y()), Point2d(b_l.x(), b_l.y()), Point2d(a_r.x(), a_r.y())},
        {b_r, b_l, a_r},
        mesh.size()});

      --i;
      --j;
    }
  }

  return mesh;
}

// compute a normal vector that is pointing the Z+ from given triangle points
Eigen::Vector3d computeFaceNormal(const std::array<Eigen::Vector3d, 3> & triangle_points)
{
  const Eigen::Vector3d v1 = triangle_points[1] - triangle_points[0];
  const Eigen::Vector3d v2 = triangle_points[2] - triangle_points[0];
  Eigen::Vector3d normal = v1.cross(v2);

  // ensure the normal is pointing upward (Z+)
  // NOTE: the triangle points are in CCW order, but there is no guarantee for
  //       lanelet's left bound and right bound to be not crossing (break the CCW order).
  if (normal.z() < 0) {
    normal = -normal;
  }

  return normal.normalized();
}

bool findNearestTriangle(
  const LaneletPolygonData & data, const Point2d & query, Triangle & triangle)
{
  std::vector<RTreeValue> result;
  data.rtree.query(boost::geometry::index::nearest(query, 1), std::back_inserter(result));
  if (!result.empty()) {
    triangle = data.mesh[result[0].second];

    return true;
  }
  return false;
}

std::vector<Triangle> searchNearestTriangle(
  const LaneletPolygonData & data, const Point2d & query, Triangle & triangle)
{
  std::vector<RTreeValue> result;
  std::vector<Triangle> data.rtree.query(
    boost::geometry::index::nearest(query, 1), std::back_inserter(result));
  if (!result.empty()) {
    triangle = data.mesh[result[0].second];

    return true;
  }
  return false;
}

// checks whether a point is located above the lanelet triangle plane
// that is closest in the perpendicular direction
/*
bool isPointAboveLaneletMesh(
  const Eigen::Vector3d & point, const lanelet::ConstLanelet & lanelet, const double & offset,
  const double & min_distance, const double & max_distance)
{
  const TriangleMesh mesh = createTriangleMeshFromLanelet(lanelet);

  if (mesh.empty()) return true;

  // Find the triangle that is closest in X-Y plane to avoid issues with vertical curves
  double closest_xy_distance = std::numeric_limits<double>::infinity();
  Eigen::Vector3d closest_normal;
  double distance_to_closest_surface = 0.0;
  bool found_valid_triangle = false;

  // First pass: find the triangle closest in X-Y plane
  for (const auto & tri : mesh) {
    const Eigen::Vector3d plane_normal_vec = computeFaceNormal(tri);

    // std::cos(M_PI / 3.0) -> 0.5;
    // in some environment, or more recent c++, it can be constexpr
    constexpr double cos_threshold = 0.5;
    const double cos_of_normal_and_z = plane_normal_vec.dot(Eigen::Vector3d::UnitZ());

    // if angle is too steep, consider as above for safety
    if (cos_of_normal_and_z < cos_threshold) {
      return true;
    }

    // For connected meshes, first check if point is inside the triangle
    const Eigen::Vector2d point_xy = point.head<2>();

    // Check if point is inside triangle using barycentric coordinates
    const Eigen::Vector2d v0 = tri[2].head<2>() - tri[0].head<2>();
    const Eigen::Vector2d v1 = tri[1].head<2>() - tri[0].head<2>();
    const Eigen::Vector2d v2 = point_xy - tri[0].head<2>();

    const double dot00 = v0.dot(v0);
    const double dot01 = v0.dot(v1);
    const double dot02 = v0.dot(v2);
    const double dot11 = v1.dot(v1);
    const double dot12 = v1.dot(v2);

    const double inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    const double u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    const double v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

    // If point is inside triangle, this is the best match - early exit
    // u and v are barycentric coordinates, if both are >= 0 and u + v <= 1,
    // the point is inside the triangle
    if ((u >= 0) && (v >= 0) && (u + v <= 1)) {
      // Point is inside the triangle, can't get better than this
      closest_xy_distance = 0.0;
      closest_normal = plane_normal_vec;

      // Calculate signed distance to this triangle's plane
      const Eigen::Vector3d vec_to_point = point - tri[0];
      distance_to_closest_surface = plane_normal_vec.dot(vec_to_point);
      found_valid_triangle = true;

      // Early exit: no need to check other triangles
      break;
    }

    // Point is outside, calculate minimum distance to triangle vertices
    double xy_dist = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 3; ++i) {
      const Eigen::Vector2d vertex_xy = tri[i].head<2>();
      const double dist_to_vertex = (point_xy - vertex_xy).norm();
      xy_dist = std::min(xy_dist, dist_to_vertex);
    }

    if (xy_dist < closest_xy_distance) {
      closest_xy_distance = xy_dist;
      closest_normal = plane_normal_vec;

      // Calculate signed distance to this triangle's plane
      const Eigen::Vector3d vec_to_point = point - tri[0];
      distance_to_closest_surface = plane_normal_vec.dot(vec_to_point);
      found_valid_triangle = true;
    }
  }

  // Handle case where no valid triangle was found
  if (!found_valid_triangle) {
    return true;  // Conservative: allow object if no valid surface found
  }

  // Calculate object bounds relative to the closest surface
  // The object extends +/- offset from its centroid along the surface normal
  const double top_distance = distance_to_closest_surface + offset;
  const double bottom_distance = distance_to_closest_surface - offset;

  // Original intention: accept if either the object's top OR bottom is within the acceptable range
  // This allows objects that are partially within the range to pass through
  if (
    (min_distance <= top_distance && top_distance <= max_distance) ||
    (min_distance <= bottom_distance && bottom_distance <= max_distance)) {
    return true;
  }

  return false;
}
*/
bool isPointAboveLaneletMesh(
  const Eigen::Vector3d & point, const LaneletPolygonData & lanelet_data, const double & offset,
  const double & min_distance, const double & max_distance,
  std::vector<std::array<Eigen::Vector3d, 3>> & nearest_triangle_list)
{
  // if (mesh.empty()) return true;

  const Point2d query_point2d(point.x(), point.y());
  Triangle nearest_triangle;
  if (!findNearestTriangle(lanelet_data, query_point2d, nearest_triangle)) {
    // consider as above for safety
    return true;
  }

  nearest_triangle_list.push_back(nearest_triangle.points3d);

  Eigen::Vector3d closest_normal;
  const Eigen::Vector3d plane_normal_vec = computeFaceNormal(nearest_triangle.points3d);

  // std::cos(M_PI / 3.0) -> 0.5;
  // in some environment, or more recent c++, it can be constexpr
  constexpr double cos_threshold = 0.5;
  const double cos_of_normal_and_z = plane_normal_vec.dot(Eigen::Vector3d::UnitZ());

  // if angle is too steep, consider as above for safety
  if (cos_of_normal_and_z < cos_threshold) {
    return true;
  }

  const Eigen::Vector3d vec_to_point = point - nearest_triangle.points3d[0];
  const double distance_to_closest_surface = plane_normal_vec.dot(vec_to_point);

  // Calculate object bounds relative to the closest surface
  // The object extends +/- offset from its centroid along the surface normal
  const double top_distance = distance_to_closest_surface + offset;
  const double bottom_distance = distance_to_closest_surface - offset;

  // Original intention: accept if either the object's top OR bottom is within the acceptable range
  // This allows objects that are partially within the range to pass through
  if (
    (min_distance <= top_distance && top_distance <= max_distance) ||
    (min_distance <= bottom_distance && bottom_distance <= max_distance)) {
    return true;
  }

  return false;
}

std::vector<geometry_msgs::msg::Pose> getLaneletCenterline(const lanelet::ConstLanelet & lanelet)
{
  bool first_elem_flag = true;
  std::vector<geometry_msgs::msg::Pose> centerline_path geometry_msgs::msg::Pose prev_p;

  for (const auto & lanelet_p : lanelet.centerline3d()) {
    geometry_msgs::msg::Pose current_p;
    current_p.position = lanelet::utils::conversion::toGeomMsgPt(lanelet_p);

    if (first_elem_flag) {
      first_elem_flag = false;
      prev_p = current_p;

      continue;
    }

    // only considers yaw of the lanelet
    const double lane_yaw = std::atan2(
      current_p.position.y - prev_p.position.y, current_p.position.x - prev_p.position.x);
    const double sin_yaw_half = std::sin(lane_yaw / 2.0);
    const double cos_yaw_half = std::cos(lane_yaw / 2.0);
    current_p.orientation.x = 0.0;
    current_p.orientation.y = 0.0;
    current_p.orientation.z = sin_yaw_half;
    current_p.orientation.w = cos_yaw_half;

    centerline_path.push_back(current_p);
    prev_p = current_p;
  }

  return centerline_path
}

bgi::rtree<RTreeValue, RtreeAlgo> buildRTreeFromMesh(const TriangleMesh & mesh)
{
  std::vector<RTreeValue> values;
  for (size_t i = 0; i < mesh.size(); ++i) {
    const auto & tri = mesh[i];
    // Compute 2D bounding box of triangle
    Box box;
    bg::assign_inverse(box);
    for (const auto & pt : tri.points2d) {
      bg::expand(box, pt);
    }
    values.push_back(RTreeValue{box, i});
  }
  return bgi::rtree<RTreeValue, RtreeAlgo>(values.begin(), values.end());
}

std::unordered_map<lanelet::Id, LaneletPolygonData> buildLaneletPolygonMap(
  lanelet::LaneletMapPtr lanelet_map_ptr,
  std::unordered_map<lanelet::Id, LaneletPolygonData> & old_map)
{
  std::unordered_map<lanelet::Id, LaneletPolygonData> new_map;

  for (const lanelet::Lanelet & lanelet : lanelet_map_ptr->laneletLayer) {
    if (
      lanelet.hasAttribute(lanelet::AttributeName::Subtype) &&
      (lanelet.attribute(lanelet::AttributeName::Subtype).value() ==
         lanelet::AttributeValueString::Road ||
       lanelet.attribute(lanelet::AttributeName::Subtype).value() == "road_shoulder")) {
      TriangleMesh mesh = createTriangleMeshFromLanelet(lanelet);

      const lanelet::Id lanelet_id = lanelet.id();
      auto it = old_map.find(lanelet_id);
      if (it == old_map.end()) {  // new lanelet
        // create r-tree for this lanelet polygons
        bgi::rtree<RTreeValue, RtreeAlgo> rtree = buildRTreeFromMesh(mesh);

        const std::vector<geometry_msgs::msg::Pose> centerline_pose_path =
          getLaneletCenterline(lanelet);

        // resample the center line points
        // in the resamplePoseVector(), flag is inverted
        // so use_akima_spline_for_xy==true means using lerp
        constexpr double path_resolution = 2.0  // I think it is in [m]
          constexpr bool use_akima_spline_for_xy = true;
        constexpr bool use_lerp_for_z = true;
        const std::vector<geometry_msgs::msg::Point> resampled_centerline_path =
          autoware::motion_utils::resamplePoseVector(
            centerline_path, path_resolution, use_akima_spline_for_xy, use_lerp_for_z);

        std::vector<CenterLinePointAndPolygonIndex> centerline_points_and_polygon_index;

        for (const auto & p : resampled_centerline_path) {
          const Point2d query_point2d(p.x, p.y);
          std::vector<RTreeValue> result;
          // std::vector<Triangle> aaaaaaaaaaaaaaaaa;
          //  we created a bounding box of triangle which is created from rectangle
          //  so we will fetch at least 2 to ensure the most nearest (= contains the point) exist
          constexpr unsingned int search_num = 2 rtree.query(
            boost::geometry::index::nearest(query, search_num), std::back_inserter(result));

          size_t assign_mesh_index;
          // find the actual polygon that contains the point
          double min_dist_from_polygon = std::numeric_limits<double>::infinity();
          for (const auto & r : result) {
            const double dist_from_polygon =
              distFromClosestPolygon(query_point2d.x, query_point2d.y, mesh[r.second]);

            // incase the point does not fall in the polygon, we will check both points
            // so, no early return here
            if (dist_from_polygon < min_dist_from_polygon) {
              min_dist_from_polygon = dist_from_polygon;
              assign_polygon = mesh[r.second];
              assign_mesh_index = r.second;
            }
          }

          // psudo (python) code
          // points_list.append((point, assign_polygon))
          centerline_points_and_polygon_index.emplace_back({query_point2d, assign_mesh_index})
        }

        // old code
        // new_map.emplace(lanelet_id, LaneletPolygonData{std::move(mesh), std::move(rtree)});
        new_map.emplace(lanelet_id, centerline_points_and_polygon_index);

      } else {
        // if ID exist, reuse it
        new_map.emplace(lanelet_id, it->second);
      }
    }
  }

  return new_map;
}

template <typename ObjsMsgType, typename ObjMsgType>
void ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::mapCallback(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr map_msg)
{
  // for the first time, run in single thread
  // NOTE: In the current Autoware, there is no dynamic loading mechanism for vector map.
  //       So this callback should run only once.
  if (!lanelet_map_ptr_) {
    lanelet_frame_id_ = map_msg->header.frame_id;
    lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_ptr_);

    lanelets_polygon_rtree_map_ =
      buildLaneletPolygonMap(lanelet_map_ptr_, lanelets_polygon_rtree_map_);
  } else {
    lanelet_frame_id_ = map_msg->header.frame_id;
    lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_ptr_);

    worker_threads_.emplace_back([this]() {
      // NOTE: Blocking does not ensured the order. so if callback get called multiple time
      //       in a short period, it might break the data.
      {
        std::lock_guard<std::mutex> lock(mtx_update_polygon_map_);

        std::unordered_map<lanelet::Id, LaneletPolygonData> tmp_map =
          buildLaneletPolygonMap(lanelet_map_ptr_, lanelets_polygon_rtree_map_);

        {
          std::lock_guard<std::mutex> lock(mtx_polygon_map_access_);
          // update current map
          lanelets_polygon_rtree_map_ = tmp_map;
        }
      }
    });
  }
}

template <typename ObjsMsgType, typename ObjMsgType>
void ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::objectCallback(
  const typename ObjsMsgType::ConstSharedPtr input_msg)
{
  stop_watch_ptr_->tic("processing_time");

  // Guard
  if (object_pub_->get_subscription_count() < 1) return;

  ObjsMsgType output_object_msg;
  output_object_msg.header = input_msg->header;

  if (!lanelet_map_ptr_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "No vector map received.");
    return;
  }

  ObjsMsgType transformed_objects;
  if (!autoware::object_recognition_utils::transformObjects(
        *input_msg, lanelet_frame_id_, tf_buffer_, transformed_objects)) {
    RCLCPP_ERROR(get_logger(), "Failed transform to %s.", lanelet_frame_id_.c_str());
    return;
  }

  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  if (!transformed_objects.objects.empty()) {
    // calculate convex hull
    const auto convex_hull = getConvexHull(transformed_objects);

    // get intersected lanelets
    std::vector<BoxAndLanelet> intersected_lanelets_with_bbox = getIntersectedLanelets(convex_hull);

    // create R-Tree with intersected_lanelets for fast search
    bgi::rtree<BoxAndLanelet, RtreeAlgo> local_rtree;
    for (const auto & bbox_and_lanelet : intersected_lanelets_with_bbox) {
      local_rtree.insert(bbox_and_lanelet);
    }

    if (filter_settings_.debug) {
      publishDebugMarkers(input_msg->header.stamp, convex_hull, intersected_lanelets_with_bbox);
    }

    {
      // block for preventing map update among the process

      // lock for preventing the update on lanelets_polygon_rtree_map_
      std::lock_guard<std::mutex> lock(mtx_polygon_map_access_);

      // filtering process
      for (size_t index = 0; index < transformed_objects.objects.size(); ++index) {
        const auto & transformed_object = transformed_objects.objects.at(index);
        const auto & input_object = input_msg->objects.at(index);
        filterObject(transformed_object, input_object, local_rtree, output_object_msg);
      }
    }
  }

  object_pub_->publish(output_object_msg);
  published_time_publisher_->publish_if_subscribed(object_pub_, output_object_msg.header.stamp);

  // Publish debug info
  const double pipeline_latency =
    std::chrono::duration<double, std::milli>(
      std::chrono::nanoseconds(
        (this->get_clock()->now() - output_object_msg.header.stamp).nanoseconds()))
      .count();
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/pipeline_latency_ms", pipeline_latency);
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/processing_time_ms", stop_watch_ptr_->toc("processing_time", true));

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = input_msg->header.stamp;
  marker.ns = "my_points";
  marker.id = 244;  // ID of marker you want to delete
  marker.action = visualization_msgs::msg::Marker::DELETE;
  // remove the markers
  point_marker_pub_->publish(marker);

  marker.ns = "my_triangles";
  marker.id = 245;  // ID of marker you want to delete
  marker.action = visualization_msgs::msg::Marker::DELETE;
  // remove the markers
  triangle_merker_pub_->publish(marker);

  marker.ns = "point_to_vertex";
  marker.id = 246;  // ID of marker you want to delete
  marker.action = visualization_msgs::msg::Marker::DELETE;
  // remove the markers
  query_to_vertex_merker_pub_->publish(marker);

  auto query_point_maker = create_point_marker(query_points_);
  auto triangle_marker = create_triangle_marker(nearest_triangles_);
  auto query_to_vertex = create_relation_line_marker(query_points_, nearest_triangles_);

  query_point_maker.header.stamp = input_msg->header.stamp;
  triangle_marker.header.stamp = input_msg->header.stamp;

  triangle_merker_pub_->publish(triangle_marker);
  point_marker_pub_->publish(query_point_maker);
  query_to_vertex_merker_pub_->publish(query_to_vertex);

  query_points_.clear();
  nearest_triangles_.clear();
}

template <typename ObjsMsgType, typename ObjMsgType>
bool ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::filterObject(
  const ObjMsgType & transformed_object, const ObjMsgType & input_object,
  const bgi::rtree<BoxAndLanelet, RtreeAlgo> & local_rtree, ObjsMsgType & output_object_msg)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const auto & label = transformed_object.classification.front().label;
  if (filter_target_.isTarget(label)) {
    // no tree, then no intersection
    if (local_rtree.empty()) {
      return false;
    }

    // create a 2D polygon from the object for querying
    Polygon2d object_polygon;
    if (utils::hasBoundingBox(transformed_object)) {
      const auto footprint = setFootprint(transformed_object);
      for (const auto & point : footprint.points) {
        const geometry_msgs::msg::Point32 point_transformed = autoware_utils::transform_point(
          point, transformed_object.kinematics.pose_with_covariance.pose);
        object_polygon.outer().emplace_back(point_transformed.x, point_transformed.y);
      }
      object_polygon.outer().push_back(object_polygon.outer().front());
    } else {
      object_polygon = getConvexHullFromObjectFootprint(transformed_object);
    }

    // create a bounding box from polygon for searching the local R-tree
    bg::model::box<bg::model::d2::point_xy<double>> bbox_of_convex_hull;
    bg::envelope(object_polygon, bbox_of_convex_hull);
    std::vector<BoxAndLanelet> candidates;
    // only use the lanelets that intersect with the object's bounding box
    local_rtree.query(bgi::intersects(bbox_of_convex_hull), std::back_inserter(candidates));

    bool filter_pass = true;
    // 1. is polygon overlap with road lanelets or shoulder lanelets
    if (filter_settings_.lanelet_xy_overlap_filter) {
      filter_pass = isObjectOverlapLanelets(transformed_object, object_polygon, candidates);
    }

    // 2. check if objects velocity is the same with the lanelet direction
    const bool orientation_not_available =
      transformed_object.kinematics.orientation_availability ==
      autoware_perception_msgs::msg::TrackedObjectKinematics::UNAVAILABLE;
    if (filter_settings_.lanelet_direction_filter && !orientation_not_available && filter_pass) {
      std::unique_ptr<ScopedTimeTrack> inner_st_ptr;
      if (time_keeper_)
        inner_st_ptr =
          std::make_unique<ScopedTimeTrack>("isSameDirectionWithLanelets", *time_keeper_);
      filter_pass = isSameDirectionWithLanelets(transformed_object, candidates);
    }

    // 3. check if the object is above the lanelets
    if (filter_settings_.lanelet_object_elevation_filter && filter_pass) {
      std::unique_ptr<ScopedTimeTrack> inner_st_ptr;
      if (time_keeper_)
        inner_st_ptr = std::make_unique<ScopedTimeTrack>("isObjectAboveLanelet", *time_keeper_);
      filter_pass = isObjectAboveLanelet(transformed_object, candidates);
    }

    // push back to output object
    if (filter_pass) {
      output_object_msg.objects.emplace_back(input_object);
      return true;
    }
  } else {
    output_object_msg.objects.emplace_back(input_object);
    return true;
  }
  return false;
}

template <typename ObjsMsgType, typename ObjMsgType>
geometry_msgs::msg::Polygon ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::setFootprint(
  const ObjMsgType & detected_object)
{
  geometry_msgs::msg::Polygon footprint;
  if (detected_object.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    const auto object_size = detected_object.shape.dimensions;
    const double x_front = object_size.x / 2.0;
    const double x_rear = -object_size.x / 2.0;
    const double y_left = object_size.y / 2.0;
    const double y_right = -object_size.y / 2.0;

    footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(x_front).y(y_left).z(0.0));
    footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(x_front).y(y_right).z(0.0));
    footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(x_rear).y(y_right).z(0.0));
    footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(x_rear).y(y_left).z(0.0));
  } else {
    footprint = detected_object.shape.footprint;
  }
  return footprint;
}

template <typename ObjsMsgType, typename ObjMsgType>
LinearRing2d ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::getConvexHull(
  const ObjsMsgType & detected_objects)
{
  MultiPoint2d candidate_points;
  for (const auto & object : detected_objects.objects) {
    const auto & pos = object.kinematics.pose_with_covariance.pose.position;
    const auto footprint = setFootprint(object);
    for (const auto & p : footprint.points) {
      candidate_points.emplace_back(p.x + pos.x, p.y + pos.y);
    }
  }
  LinearRing2d convex_hull;
  bg::convex_hull(candidate_points, convex_hull);

  if (filter_settings_.lanelet_extra_margin > 0) {
    return expandPolygon(convex_hull, filter_settings_.lanelet_extra_margin);
  }
  return convex_hull;
}

template <typename ObjsMsgType, typename ObjMsgType>
Polygon2d ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::getConvexHullFromObjectFootprint(
  const ObjMsgType & object)
{
  MultiPoint2d candidate_points;
  const auto & pos = object.kinematics.pose_with_covariance.pose.position;
  const auto footprint = setFootprint(object);

  for (const auto & p : footprint.points) {
    candidate_points.emplace_back(p.x + pos.x, p.y + pos.y);
  }

  Polygon2d convex_hull;
  bg::convex_hull(candidate_points, convex_hull);

  return convex_hull;
}

// fetch the intersected candidate lanelets with bounding box and then
// check the intersections among the lanelets and the convex hull
template <typename ObjsMsgType, typename ObjMsgType>
std::vector<BoxAndLanelet> ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::getIntersectedLanelets(
  const LinearRing2d & convex_hull)
{
  std::vector<BoxAndLanelet> intersected_lanelets_with_bbox;

  // convert convex_hull to a 2D bounding box for searching in the LaneletMap
  bg::model::box<bg::model::d2::point_xy<double>> bbox_of_convex_hull;
  bg::envelope(convex_hull, bbox_of_convex_hull);
  const lanelet::BoundingBox2d bbox2d(
    lanelet::BasicPoint2d(
      bg::get<bg::min_corner, 0>(bbox_of_convex_hull),
      bg::get<bg::min_corner, 1>(bbox_of_convex_hull)),
    lanelet::BasicPoint2d(
      bg::get<bg::max_corner, 0>(bbox_of_convex_hull),
      bg::get<bg::max_corner, 1>(bbox_of_convex_hull)));

  const lanelet::Lanelets candidate_lanelets = lanelet_map_ptr_->laneletLayer.search(bbox2d);
  for (const auto & lanelet : candidate_lanelets) {
    // only check the road lanelets and road shoulder lanelets
    if (
      lanelet.hasAttribute(lanelet::AttributeName::Subtype) &&
      (lanelet.attribute(lanelet::AttributeName::Subtype).value() ==
         lanelet::AttributeValueString::Road ||
       lanelet.attribute(lanelet::AttributeName::Subtype).value() == "road_shoulder")) {
      if (bg::intersects(convex_hull, lanelet.polygon2d().basicPolygon())) {
        // create bbox using boost for making the R-tree in later phase
        auto polygon = getPolygon(lanelet);
        Box boost_bbox;
        bg::envelope(polygon, boost_bbox);

        intersected_lanelets_with_bbox.emplace_back(
          std::make_pair(boost_bbox, PolygonAndLanelet{polygon, lanelet}));
      }
    }
  }

  return intersected_lanelets_with_bbox;
}

template <typename ObjsMsgType, typename ObjMsgType>
lanelet::BasicPolygon2d ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::getPolygon(
  const lanelet::ConstLanelet & lanelet)
{
  if (filter_settings_.lanelet_extra_margin <= 0) {
    return lanelet.polygon2d().basicPolygon();
  }

  auto lanelet_polygon = lanelet.polygon2d().basicPolygon();
  Polygon2d polygon;
  bg::assign_points(polygon, lanelet_polygon);

  bg::correct(polygon);
  auto polygon_result = expandPolygon(polygon.outer(), filter_settings_.lanelet_extra_margin);
  lanelet::BasicPolygon2d result;

  bg::assign_points(result, polygon_result);

  return result;
}

template <typename ObjsMsgType, typename ObjMsgType>
bool ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::isObjectOverlapLanelets(
  const ObjMsgType & object, const Polygon2d & polygon,
  const std::vector<BoxAndLanelet> & lanelet_candidates)
{
  // if object has bounding box, use polygon overlap
  if (utils::hasBoundingBox(object)) {
    return isPolygonOverlapLanelets(polygon, lanelet_candidates);
  } else {
    for (const auto & point : object.shape.footprint.points) {
      const geometry_msgs::msg::Point32 point_transformed =
        autoware_utils::transform_point(point, object.kinematics.pose_with_covariance.pose);

      for (const auto & candidate : lanelet_candidates) {
        if (isInPolygon(point_transformed.x, point_transformed.y, candidate.second.polygon, 0.0)) {
          return true;
        }
      }
    }
    return false;
  }
}

template <typename ObjsMsgType, typename ObjMsgType>
bool ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::isPolygonOverlapLanelets(
  const Polygon2d & polygon, const std::vector<BoxAndLanelet> & lanelet_candidates)
{
  for (const auto & box_and_lanelet : lanelet_candidates) {
    if (!bg::disjoint(polygon, box_and_lanelet.second.polygon)) {
      return true;
    }
  }

  return false;
}

template <typename ObjsMsgType, typename ObjMsgType>
bool ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::isSameDirectionWithLanelets(
  const ObjMsgType & object, const std::vector<BoxAndLanelet> & lanelet_candidates)
{
  const double object_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  const double object_velocity_norm = std::hypot(
    object.kinematics.twist_with_covariance.twist.linear.x,
    object.kinematics.twist_with_covariance.twist.linear.y);
  const double object_velocity_yaw = std::atan2(
                                       object.kinematics.twist_with_covariance.twist.linear.y,
                                       object.kinematics.twist_with_covariance.twist.linear.x) +
                                     object_yaw;

  if (object_velocity_norm < filter_settings_.lanelet_direction_filter_object_speed_threshold) {
    return true;
  }

  for (const auto & box_and_lanelet : lanelet_candidates) {
    const bool is_in_lanelet =
      isInPolygon(object.kinematics.pose_with_covariance.pose, box_and_lanelet.second.polygon, 0.0);
    if (!is_in_lanelet) {
      continue;
    }

    const double lane_yaw = lanelet::utils::getLaneletAngle(
      box_and_lanelet.second.lanelet, object.kinematics.pose_with_covariance.pose.position);
    const double delta_yaw = object_velocity_yaw - lane_yaw;
    const double normalized_delta_yaw = autoware_utils::normalize_radian(delta_yaw);
    const double abs_norm_delta_yaw = std::fabs(normalized_delta_yaw);

    if (abs_norm_delta_yaw < filter_settings_.lanelet_direction_filter_velocity_yaw_threshold) {
      return true;
    }
  }

  return false;
}

// this is the old one with polygon search with rtree
/*
template <typename ObjsMsgType, typename ObjMsgType>
bool ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::isObjectAboveLanelet(
  const ObjMsgType & object, const std::vector<BoxAndLanelet> & lanelet_candidates)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // assuming the positions are already the center of the cluster (convex hull)
  // for an exact calculation of the center from the points,
  // we should use autoware_utils::transform_point before computing the cluster
  const double cx = object.kinematics.pose_with_covariance.pose.position.x;
  const double cy = object.kinematics.pose_with_covariance.pose.position.y;
  const double cz = object.kinematics.pose_with_covariance.pose.position.z;
  // use the centroid as a query point
  const Eigen::Vector3d centroid(cx, cy, cz);
  const double half_dim_z = object.shape.dimensions.z * 0.5;

  lanelet::ConstLanelet nearest_lanelet;
  double closest_lanelet_z_dist = std::numeric_limits<double>::infinity();
  double min_dist_xy = std::numeric_limits<double>::infinity();

  // search for the nearest lanelet along the z-axis in case roads are layered
  for (const auto & candidate_lanelet : lanelet_candidates) {
    const lanelet::ConstLanelet llt = candidate_lanelet.second.lanelet;
    const lanelet::ConstLineString3d line = llt.leftBound();
    if (line.empty()) continue;

    /////////////////////////////////////////////////////////
    // legacy
    // assuming the roads have enough height difference to distinguish each other
    // const double diff_z = cz - line[0].z();
    // const double dist_z = diff_z * diff_z;

    // use the closest lanelet in z axis
    // if (dist_z < closest_lanelet_z_dist) {
    //  closest_lanelet_z_dist = dist_z;
    //  nearest_lanelet = llt;
    //}
    /////////////////////////////////////////////////////////

    const double diff_z = cz - line[0].z();
    if (diff_z * diff_z > elevation_min_z_dist_sq_) continue;

    // search the most nearest lanelet
    const dist_xy = distFromClosestPolygon(cx, xy, candidate_lanelet.second.polygon);
    if (dist_xy = 0.0) {
      // if distance is 0.0, it is inside the polygon
      nearest_lanelet = llt;
      break;
    }

    if (dist_xy < min_dist_xy) {
      min_dist_xy = dist_xy;
      nearest_lanelet = llt;
    }
  }

  auto it = lanelets_polygon_rtree_map_.find(nearest_lanelet.id());
  if (it == lanelets_polygon_rtree_map_.end()) {
    // return true for safety
    // also handles the case when map has been updated while the filtering process
    return true;
  }

  // return isPointAboveLaneletMesh(
  //   centroid, nearest_lanelet, half_dim_z, filter_settings_.min_elevation_threshold,
  //   filter_settings_.max_elevation_threshold);

  query_points_.push_back(centroid);

  return isPointAboveLaneletMesh(
    centroid, it->second, half_dim_z, filter_settings_.min_elevation_threshold,
    filter_settings_.max_elevation_threshold, nearest_triangles_);
}*/
template <typename ObjsMsgType, typename ObjMsgType>
bool ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::isObjectAboveLanelet(
  const ObjMsgType & object, const std::vector<BoxAndLanelet> & lanelet_candidates)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // assuming the positions are already the center of the cluster (convex hull)
  // for an exact calculation of the center from the points,
  // we should use autoware_utils::transform_point before computing the cluster
  const double cx = object.kinematics.pose_with_covariance.pose.position.x;
  const double cy = object.kinematics.pose_with_covariance.pose.position.y;
  const double cz = object.kinematics.pose_with_covariance.pose.position.z;
  // use the centroid as a query point
  const Eigen::Vector3d centroid(cx, cy, cz);
  const double half_dim_z = object.shape.dimensions.z * 0.5;

  lanelet::ConstLanelet nearest_lanelet;
  double closest_lanelet_z_dist = std::numeric_limits<double>::infinity();
  double min_dist_xy = std::numeric_limits<double>::infinity();

  // search for the nearest lanelet along the z-axis in case roads are layered
  for (const auto & candidate_lanelet : lanelet_candidates) {
    const lanelet::ConstLanelet llt = candidate_lanelet.second.lanelet;
    const lanelet::Id lanelet_id = lanelet.id();
    auto it = lanelets_centerline_map_.find(nearest_lanelet.id());

    if (it == lanelets_centerline_map_.end()) {
      // return true for safety
      // also handles the case when map has been updated while the filtering process
      return true;
    }

    constexpr for (const auto & point_and_index : it)
    {
      Point2d clp = point_and_index->first;
      const float dx = cx - clp.x;
      const float dx = cx - clp.y;
      const float dist_sq = dx * dx + dy * dy;
    }

    const lanelet::ConstLineString3d line = llt.leftBound();
    if (line.empty()) continue;

    /////////////////////////////////////////////////////////
    // legacy
    // assuming the roads have enough height difference to distinguish each other
    // const double diff_z = cz - line[0].z();
    // const double dist_z = diff_z * diff_z;

    // use the closest lanelet in z axis
    // if (dist_z < closest_lanelet_z_dist) {
    //  closest_lanelet_z_dist = dist_z;
    //  nearest_lanelet = llt;
    //}
    /////////////////////////////////////////////////////////

    const double diff_z = cz - line[0].z();
    if (diff_z * diff_z > elevation_min_z_dist_sq_) continue;

    // search the most nearest lanelet
    const dist_xy = distFromClosestPolygon(cx, xy, candidate_lanelet.second.polygon);
    if (dist_xy = 0.0) {
      // if distance is 0.0, it is inside the polygon
      nearest_lanelet = llt;
      break;
    }

    if (dist_xy < min_dist_xy) {
      min_dist_xy = dist_xy;
      nearest_lanelet = llt;
    }
  }

  auto it = lanelets_polygon_rtree_map_.find(nearest_lanelet.id());
  if (it == lanelets_polygon_rtree_map_.end()) {
    // return true for safety
    // also handles the case when map has been updated while the filtering process
    return true;
  }

  // return isPointAboveLaneletMesh(
  //   centroid, nearest_lanelet, half_dim_z, filter_settings_.min_elevation_threshold,
  //   filter_settings_.max_elevation_threshold);

  query_points_.push_back(centroid);

  return isPointAboveLaneletMesh(
    centroid, it->second, half_dim_z, filter_settings_.min_elevation_threshold,
    filter_settings_.max_elevation_threshold, nearest_triangles_);
}

// explicit instantiation
template class ObjectLaneletFilterBase<
  autoware_perception_msgs::msg::DetectedObjects, autoware_perception_msgs::msg::DetectedObject>;
template class ObjectLaneletFilterBase<
  autoware_perception_msgs::msg::TrackedObjects, autoware_perception_msgs::msg::TrackedObject>;

}  // namespace lanelet_filter
}  // namespace autoware::detected_object_validation
