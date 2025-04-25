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

#include "lanelet_filter.hpp"

#include "autoware/object_recognition_utils/object_recognition_utils.hpp"
#include "autoware_lanelet2_extension/utility/message_conversion.hpp"
#include "autoware_lanelet2_extension/utility/query.hpp"
#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_utils/system/time_keeper.hpp"

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::detected_object_validation
{
namespace lanelet_filter
{
using autoware_utils::ScopedTimeTrack;
using TriangleMesh = std::vector<std::array<Eigen::Vector3d, 3>>;

ObjectLaneletFilterNode::ObjectLaneletFilterNode(const rclcpp::NodeOptions & node_options)
: Node("object_lanelet_filter_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
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
  filter_settings_.polygon_overlap_filter =
    declare_parameter<bool>("filter_settings.polygon_overlap_filter.enabled");
  filter_settings_.lanelet_direction_filter =
    declare_parameter<bool>("filter_settings.lanelet_direction_filter.enabled");
  filter_settings_.lanelet_direction_filter_velocity_yaw_threshold =
    declare_parameter<double>("filter_settings.lanelet_direction_filter.velocity_yaw_threshold");
  filter_settings_.lanelet_direction_filter_object_speed_threshold =
    declare_parameter<double>("filter_settings.lanelet_direction_filter.object_speed_threshold");
  filter_settings_.debug = declare_parameter<bool>("filter_settings.debug");
  filter_settings_.lanelet_extra_margin =
    declare_parameter<double>("filter_settings.lanelet_extra_margin");
  filter_settings_.use_height_threshold =
    declare_parameter<bool>("filter_settings.use_height_threshold");
  filter_settings_.max_height_threshold =
    declare_parameter<double>("filter_settings.max_height_threshold");
  filter_settings_.min_height_threshold =
    declare_parameter<double>("filter_settings.min_height_threshold");

  // Set publisher/subscriber
  map_sub_ = this->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&ObjectLaneletFilterNode::mapCallback, this, _1));
  object_sub_ = this->create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
    "input/object", rclcpp::QoS{1}, std::bind(&ObjectLaneletFilterNode::objectCallback, this, _1));
  object_pub_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "output/object", rclcpp::QoS{1});

  debug_publisher_ =
    std::make_unique<autoware_utils::DebugPublisher>(this, "object_lanelet_filter");
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  if (filter_settings_.debug) {
    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/debug/marker", rclcpp::QoS{1});
  }

  // debug
  debug_query_point_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("~/debug/marker", 10);

  detailed_processing_time_publisher_ =
    this->create_publisher<autoware_utils::ProcessingTimeDetail>(
      "~/debug/processing_time_detail_ms", 1);
  auto time_keeper = autoware_utils::TimeKeeper(detailed_processing_time_publisher_);
  time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(time_keeper);
}

bool isInPolygon(
  const geometry_msgs::msg::Pose & current_pose, const lanelet::BasicPolygon2d & polygon,
  const double radius)
{
  constexpr double eps = 1.0e-9;
  const lanelet::BasicPoint2d p(current_pose.position.x, current_pose.position.y);
  return boost::geometry::distance(p, polygon) < radius + eps;
}

bool isInPolygon(
  const double & x, const double & y, const lanelet::BasicPolygon2d & polygon,
  const double radius)
{
  constexpr double eps = 1.0e-9;
  const lanelet::BasicPoint2d p(x, y);
  return boost::geometry::distance(p, polygon) < radius + eps;
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

TriangleMesh createTriangleMeshFromLanelet(const lanelet::ConstLanelet & lanelet) {
  TriangleMesh mesh;

  const lanelet::ConstLineString3d & left = lanelet.leftBound();
  const lanelet::ConstLineString3d & right = lanelet.rightBound();

  // bounds must have same number of points
  // if not the same, only check the first shared points
  size_t n = std::min(left.size(), right.size());
  if (n < 2) return mesh;

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
    mesh.push_back({a_l, b_l, a_r});
    mesh.push_back({b_l, b_r, a_r});
  }

  return mesh;
}

// compute a normal vector that is pointing the Z+ from given triangle points
Eigen::Vector3d computeFaceNormal(const std::array<Eigen::Vector3d, 3>& triangle_points) {
    Eigen::Vector3d v1 = triangle_points[1] - triangle_points[0];
    Eigen::Vector3d v2 = triangle_points[2] - triangle_points[0];
    Eigen::Vector3d normal = v1.cross(v2);

    // ensure the normal is pointing upward (Z+)
    if (normal.z() < 0) {
        normal = -normal;
    }

    return normal.normalized();
}

// check the query point is whether above the lanelet's Linestring segment's plane
bool isPointAboveLaneletSegment(
  const Eigen::Vector3d& query_point, const lanelet::ConstLineString3d & line,
  const double offset = 0.0)
{
  if (line.size() < 2) {
    return true;
  }

  // search the nearest point from the query point
  //double min_point_dist = std::numeric_limits<double>::infinity();
  //size_t min_index = 0;
  //for (size_t i = 0; i < line.size(); ++i) {
  //  const double diff_x  = (line[i].x()-query_point.x());
  //  const double diff_y  = (line[i].y()-query_point.y());
  //  const double diff_z  = (line[i].z()-query_point.z());
  //  const double dist = diff_x*diff_x+diff_y*diff_y+diff_z*diff_z;
  //  if (dist < min_point_dist) {
  //    min_point_dist = dist;
  //    min_index = i;
  //  }
  //}

  double min_dist = std::numeric_limits<double>::infinity();
  double min_abs_dist = std::numeric_limits<double>::infinity();
  // calculate the projection and use the nearest plane to
  //std::cout << "line size: " << line.size() << std::endl;

  // use the two segments which is created by the nearest point
  //for (size_t i = std::max(static_cast<size_t>(0), min_index-1); i + 1 < line.size() && i <= min_index+1; ++i) {
  for (size_t i = 0; i < line.size()-1; ++i) {
    //std::cout << "(" << i << ", " << i+1 << ")" << std::endl;
    const lanelet::ConstPoint3d & a = line[i];
    const lanelet::ConstPoint3d & b = line[i + 1];

    const Eigen::Vector3d pa(a.x(), a.y(), a.z());
    const Eigen::Vector3d pb(b.x(), b.y(), b.z());

    Eigen::Vector3d ab_unit_vec = pb - pa;
    if (ab_unit_vec.norm() < 1e-6) continue;
    ab_unit_vec = ab_unit_vec.normalized();

    const Eigen::Vector3d up(0, 0, 1);
    Eigen::Vector3d side = ab_unit_vec.cross(up);
    // in case of ab angled near to z axis
    if (side.norm() < 1e-6) continue;

    // calc normal vector of x-y plane that is tilted to ab
    Eigen::Vector3d normal = side.cross(ab_unit_vec).normalized();
    // ensure normal vector towards +Z
    if (normal.z() < 0) normal *= -1;

    // adjust the query point if needed by the offset
    // this is aiming the case there are outlier points in the cluster with relative large z value
    // since the current euclidean cluster is doing in the work in 2D,
    // cluster may contain some relatively large z points compare to the centroid
    Eigen::Vector3d aq = (query_point + offset * normal) - pa;
    double signed_dist = aq.dot(normal);

    // search the nearest face by the 
    if (std::abs(signed_dist) < min_abs_dist) {
      min_abs_dist = std::abs(signed_dist);
      min_dist = signed_dist;
    }
  }

  // if we can't find 
  if (min_abs_dist == std::numeric_limits<double>::infinity()) {
    return true;
  }

  //std::cout << closest_plane_normal_vector_dist << std::endl;
  if (min_dist < 0) return false;
  else return true;
}

// checks whether a point is located above the lanelet triangle plane
// that is closest in the perpendicular direction
bool isPointAboveLaneletMesh(const Eigen::Vector3d & point, const lanelet::ConstLanelet & lanelet,
  const double offset = 0.0)
{
  TriangleMesh mesh = createTriangleMeshFromLanelet(lanelet);

  if (mesh.size() == 0) return true;

  double min_dist = std::numeric_limits<double>::infinity();
  double min_abs_dist = std::numeric_limits<double>::infinity();
  for (const auto& tri : mesh) {
    Eigen::Vector3d plane_normal_vec = computeFaceNormal(tri);
    // use offset for outlier points to take into account
    Eigen::Vector3d vec_to_point = (point + offset * plane_normal_vec) - tri[0];
    double signed_dist = plane_normal_vec.dot(vec_to_point);

    const double abs_dist = std::abs(signed_dist);
    if (abs_dist < min_abs_dist) {
      min_abs_dist = abs_dist;
      min_dist = signed_dist;
    }
  }

  return min_dist > 0;
}


void ObjectLaneletFilterNode::mapCallback(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr map_msg)
{
  lanelet_frame_id_ = map_msg->header.frame_id;
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_ptr_);
}

void ObjectLaneletFilterNode::objectCallback(
  const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr input_msg)
{
  stop_watch_ptr_->tic("processing_time");

  // Guard
  if (object_pub_->get_subscription_count() < 1) return;

  autoware_perception_msgs::msg::DetectedObjects output_object_msg;
  output_object_msg.header = input_msg->header;

  if (!lanelet_map_ptr_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "No vector map received.");
    return;
  }

  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  autoware_perception_msgs::msg::DetectedObjects transformed_objects;
  if (!autoware::object_recognition_utils::transformObjects(
        *input_msg, lanelet_frame_id_, tf_buffer_, transformed_objects)) {
    RCLCPP_ERROR(get_logger(), "Failed transform to %s.", lanelet_frame_id_.c_str());
    return;
  }
  // vehicle base pose :map -> base_link
  if (filter_settings_.use_height_threshold) {
    try {
      ego_base_height_ = tf_buffer_
                           .lookupTransform(
                             lanelet_frame_id_, "base_link", transformed_objects.header.stamp,
                             rclcpp::Duration::from_seconds(0.5))
                           .transform.translation.z;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to get transform: " << ex.what());
      return;
    }
  }

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
    // filtering process
    for (size_t index = 0; index < transformed_objects.objects.size(); ++index) {
      const auto & transformed_object = transformed_objects.objects.at(index);
      const auto & input_object = input_msg->objects.at(index);
      filterObject(transformed_object, input_object, local_rtree, output_object_msg);
    }
  }

  // debug publish of query points
  pcl::PointCloud<pcl::PointXYZRGB> query_points_in_map_ptr_rgb;
  for (std::size_t i = 0; i < query_points_.size(); i++) {
    pcl::PointXYZRGB point;
    point.x = query_points_[i].x();
    point.y = query_points_[i].y();
    point.z = query_points_[i].z();
    point.r = 255;
    point.g = 0;
    point.b = 0;
    query_points_in_map_ptr_rgb.points.push_back(point);
  }
  sensor_msgs::msg::PointCloud2 query_points_in_map;
  pcl::toROSMsg(query_points_in_map_ptr_rgb, query_points_in_map);
  query_points_in_map.header.stamp = output_object_msg.header.stamp;
  query_points_in_map.header.frame_id = output_object_msg.header.frame_id;
  query_points_.clear();
  debug_query_point_pub_->publish(query_points_in_map);

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
}

bool ObjectLaneletFilterNode::filterObject(
  const autoware_perception_msgs::msg::DetectedObject & transformed_object,
  const autoware_perception_msgs::msg::DetectedObject & input_object,
  const bgi::rtree<BoxAndLanelet, RtreeAlgo> & local_rtree,
  autoware_perception_msgs::msg::DetectedObjects & output_object_msg)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const auto & label = transformed_object.classification.front().label;
  if (filter_target_.isTarget(label)) {
    // no tree, then no intersection
    if (local_rtree.empty()) {
      return false;
    }

    // 0. check height threshold
    if (filter_settings_.use_height_threshold) {
      const double object_height = transformed_object.shape.dimensions.z;
      if (
        object_height > ego_base_height_ + filter_settings_.max_height_threshold ||
        object_height < ego_base_height_ + filter_settings_.min_height_threshold) {
        return false;
      }
    }

    bool filter_pass = true;

    // 1. check if objects velocity is the same with the lanelet direction
    const bool orientation_not_available =
      transformed_object.kinematics.orientation_availability ==
      autoware_perception_msgs::msg::TrackedObjectKinematics::UNAVAILABLE;
    if (filter_settings_.lanelet_direction_filter && !orientation_not_available) {
      const bool is_same_direction = isSameDirectionWithLanelets(transformed_object, local_rtree);
      filter_pass = filter_pass && is_same_direction;
    }

    if (filter_pass) {
      // 2. is polygon overlap with road lanelets or shoulder lanelets
      if (filter_settings_.polygon_overlap_filter) {
        const bool is_polygon_overlap = isObjectOverlapLanelets(transformed_object, local_rtree);
        filter_pass = filter_pass && is_polygon_overlap;
      }

      // push back to output object
      if (filter_pass) {
        output_object_msg.objects.emplace_back(input_object);
        return true;
      }
    }
  } else {
    output_object_msg.objects.emplace_back(input_object);
    return true;
  }
  return false;
}

geometry_msgs::msg::Polygon ObjectLaneletFilterNode::setFootprint(
  const autoware_perception_msgs::msg::DetectedObject & detected_object)
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

LinearRing2d ObjectLaneletFilterNode::getConvexHull(
  const autoware_perception_msgs::msg::DetectedObjects & detected_objects)
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

LinearRing2d ObjectLaneletFilterNode::getConvexHullFromObjectFootprint(
  const autoware_perception_msgs::msg::DetectedObject & object)
{
  MultiPoint2d candidate_points;
  const auto & pos = object.kinematics.pose_with_covariance.pose.position;
  const auto footprint = setFootprint(object);

  for (const auto & p : footprint.points) {
    candidate_points.emplace_back(p.x + pos.x, p.y + pos.y);
  }

  LinearRing2d convex_hull;
  bg::convex_hull(candidate_points, convex_hull);

  return convex_hull;
}

// fetch the intersected candidate lanelets with bounding box and then
// check the intersections among the lanelets and the convex hull
std::vector<BoxAndLanelet> ObjectLaneletFilterNode::getIntersectedLanelets(
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

lanelet::BasicPolygon2d ObjectLaneletFilterNode::getPolygon(const lanelet::ConstLanelet & lanelet)
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

bool ObjectLaneletFilterNode::isObjectOverlapLanelets(
  const autoware_perception_msgs::msg::DetectedObject & object,
  const bgi::rtree<BoxAndLanelet, RtreeAlgo> & local_rtree)
{
  // if object has bounding box, use polygon overlap
  if (utils::hasBoundingBox(object)) {
    Polygon2d polygon;
    const auto footprint = setFootprint(object);
    for (const auto & point : footprint.points) {
      const geometry_msgs::msg::Point32 point_transformed =
        autoware_utils::transform_point(point, object.kinematics.pose_with_covariance.pose);
      polygon.outer().emplace_back(point_transformed.x, point_transformed.y);
    }
    polygon.outer().push_back(polygon.outer().front());

    return isPolygonOverlapLanelets(polygon, local_rtree);
  } else {
    const LinearRing2d object_convex_hull = getConvexHullFromObjectFootprint(object);

    // create bounding box to search in the rtree
    std::vector<BoxAndLanelet> candidates;
    bg::model::box<bg::model::d2::point_xy<double>> bbox;
    bg::envelope(object_convex_hull, bbox);
    local_rtree.query(bgi::intersects(bbox), std::back_inserter(candidates));

    // for centroid
    double cx = 0.0;
    double cy = 0.0;
    double cz = 0.0;
    // used for approximating max radius of the points from the centroid
    double max_z = std::numeric_limits<double>::min();

    bool point_in_polygon = false;
    std::vector<lanelet::ConstLanelet> lanelets_containing_cluster_points;

    // if object do not have bounding box, check each footprint is inside polygon
    for (const auto & point : object.shape.footprint.points) {
      const geometry_msgs::msg::Point32 point_transformed =
        autoware_utils::transform_point(point, object.kinematics.pose_with_covariance.pose);
      //geometry_msgs::msg::Pose point2d;
      //point2d.position.x = point_transformed.x;
      //point2d.position.y = point_transformed.y;
      cx += point_transformed.x;
      cy += point_transformed.y;
      cz += point_transformed.z;
      max_z = std::max(max_z, static_cast<double>(point_transformed.z));

      for (const auto & candidate : candidates) {
        if (isInPolygon(point_transformed.x, point_transformed.y, candidate.second.polygon, 0.0)) {
          // original implementation
          //return true;
          // store the lanelet for checking the face
          lanelets_containing_cluster_points.push_back(candidate.second.lanelet);
          point_in_polygon = true;
          break;
        }
      }
    }

    if (point_in_polygon) {
      // check if the centroid of the cluster is above the lanelet

      const int point_num = object.shape.footprint.points.size();
      cx /= point_num;
      cy /= point_num;
      cz /= point_num;

      const Eigen::Vector3d query_point(cx, cy, cz);
      query_points_.push_back(query_point);

      std::optional<lanelet::ConstLanelet> nearest_lanelet;
      double closest_lanelet_z_dist = std::numeric_limits<double>::infinity();

      // search the nearest lanelet in the z axis in case roads are layered
      for (const auto & candidate_lanelet : lanelets_containing_cluster_points) {
        const lanelet::ConstLineString3d line = candidate_lanelet.leftBound();
        if (line.size() == 0) continue;

        // assuming the roads have enough height to distinguish each others
        const double diff_z = cz - line[0].z();
        const double dist_z = diff_z * diff_z;

        // use the closest lanelet in z axis
        if (dist_z < closest_lanelet_z_dist) {
          closest_lanelet_z_dist = dist_z;
          nearest_lanelet = candidate_lanelet;
        }
      }

      if (false && nearest_lanelet) {
        const double offset = std::abs(max_z-cz);

        if (true){
          std::cout << "checking by segment" << std::endl;
          // we might check both sides and then `AND` the result
          return isPointAboveLaneletSegment(query_point, nearest_lanelet.value().leftBound(), offset);
        } else {
          std::cout << "checking by triangle polygon" << std::endl;
          return isPointAboveLaneletMesh(query_point, nearest_lanelet.value(), offset);
        }
      } else {
        std::cout << "no lanelet checking" << std::endl;
        // use the result of isInPolygon
        return true;
      }
    }

    return false;
  }
}

bool ObjectLaneletFilterNode::isPolygonOverlapLanelets(
  const Polygon2d & polygon, const bgi::rtree<BoxAndLanelet, RtreeAlgo> & local_rtree)
{
  // create a bounding box from polygon for searching the local R-tree
  std::vector<BoxAndLanelet> candidates;
  bg::model::box<bg::model::d2::point_xy<double>> bbox_of_convex_hull;
  bg::envelope(polygon, bbox_of_convex_hull);
  local_rtree.query(bgi::intersects(bbox_of_convex_hull), std::back_inserter(candidates));

  for (const auto & box_and_lanelet : candidates) {
    if (!bg::disjoint(polygon, box_and_lanelet.second.polygon)) {
      return true;
    }
  }

  return false;
}

bool ObjectLaneletFilterNode::isSameDirectionWithLanelets(
  const autoware_perception_msgs::msg::DetectedObject & object,
  const bgi::rtree<BoxAndLanelet, RtreeAlgo> & local_rtree)
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

  // we can not query by points, so create a small bounding box
  // eps is not determined by any specific criteria; just a guess
  constexpr double eps = 1.0e-6;
  std::vector<BoxAndLanelet> candidates;
  const Point2d min_corner(
    object.kinematics.pose_with_covariance.pose.position.x - eps,
    object.kinematics.pose_with_covariance.pose.position.y - eps);
  const Point2d max_corner(
    object.kinematics.pose_with_covariance.pose.position.x + eps,
    object.kinematics.pose_with_covariance.pose.position.y + eps);
  const Box bbox(min_corner, max_corner);

  local_rtree.query(bgi::intersects(bbox), std::back_inserter(candidates));

  for (const auto & box_and_lanelet : candidates) {
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

}  // namespace lanelet_filter
}  // namespace autoware::detected_object_validation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::detected_object_validation::lanelet_filter::ObjectLaneletFilterNode)
