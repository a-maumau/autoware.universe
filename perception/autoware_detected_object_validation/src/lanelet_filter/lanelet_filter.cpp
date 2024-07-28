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

#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/system/time_keeper.hpp"
#include "autoware_lanelet2_extension/utility/message_conversion.hpp"
#include "autoware_lanelet2_extension/utility/query.hpp"
#include "object_recognition_utils/object_recognition_utils.hpp"

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/Polygon.h>

namespace autoware::detected_object_validation
{
namespace lanelet_filter
{
ObjectLaneletFilterNode::ObjectLaneletFilterNode(const rclcpp::NodeOptions & node_options)
: Node("object_lanelet_filter_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;

  // Set parameters
  filter_target_.UNKNOWN = declare_parameter<bool>("filter_target_label.UNKNOWN", false);
  filter_target_.CAR = declare_parameter<bool>("filter_target_label.CAR", false);
  filter_target_.TRUCK = declare_parameter<bool>("filter_target_label.TRUCK", false);
  filter_target_.BUS = declare_parameter<bool>("filter_target_label.BUS", false);
  filter_target_.TRAILER = declare_parameter<bool>("filter_target_label.TRAILER", false);
  filter_target_.MOTORCYCLE = declare_parameter<bool>("filter_target_label.MOTORCYCLE", false);
  filter_target_.BICYCLE = declare_parameter<bool>("filter_target_label.BICYCLE", false);
  filter_target_.PEDESTRIAN = declare_parameter<bool>("filter_target_label.PEDESTRIAN", false);
  // Set filter settings
  filter_settings_.polygon_overlap_filter =
    declare_parameter<bool>("filter_settings.polygon_overlap_filter.enabled");
  filter_settings_.lanelet_direction_filter =
    declare_parameter<bool>("filter_settings.lanelet_direction_filter.enabled");
  filter_settings_.lanelet_direction_filter_velocity_yaw_threshold =
    declare_parameter<double>("filter_settings.lanelet_direction_filter.velocity_yaw_threshold");
  filter_settings_.lanelet_direction_filter_object_speed_threshold =
    declare_parameter<double>("filter_settings.lanelet_direction_filter.object_speed_threshold");

  // Set publisher/subscriber
  map_sub_ = this->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&ObjectLaneletFilterNode::mapCallback, this, _1));
  object_sub_ = this->create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
    "input/object", rclcpp::QoS{1}, std::bind(&ObjectLaneletFilterNode::objectCallback, this, _1));
  object_pub_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "output/object", rclcpp::QoS{1});

  debug_publisher_ =
    std::make_unique<autoware::universe_utils::DebugPublisher>(this, "object_lanelet_filter");
  published_time_publisher_ =
    std::make_unique<autoware::universe_utils::PublishedTimePublisher>(this);

  publisher_ =
      this->create_publisher<autoware::universe_utils::ProcessingTimeDetail>("~/debug/lanelet_filter_processing_time_detail_ms", rclcpp::QoS{1});
  time_keeper_ = std::make_shared<autoware::universe_utils::TimeKeeper>(publisher_, &std::cerr);
  //timer_ =
  //    create_wall_timer(std::chrono::seconds(1), std::bind(&ObjectLaneletFilterNode::objectCallback, this));
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
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // Guard
  if (object_pub_->get_subscription_count() < 1) return;

  autoware_perception_msgs::msg::DetectedObjects output_object_msg;
  output_object_msg.header = input_msg->header;

  if (!lanelet_map_ptr_) {
    RCLCPP_ERROR(get_logger(), "No vector map received.");
    return;
  }
  autoware_perception_msgs::msg::DetectedObjects transformed_objects;
  if (!object_recognition_utils::transformObjects(
        *input_msg, lanelet_frame_id_, tf_buffer_, transformed_objects)) {
    RCLCPP_ERROR(get_logger(), "Failed transform to %s.", lanelet_frame_id_.c_str());
    return;
  }

  // calculate convex hull
  const auto convex_hull = getConvexHull(transformed_objects);

  // get intersected lanelets
  //lanelet::ConstLanelets intersected_lanelets = getIntersectedLanelets(convex_hull);
  std::vector<BoxAndLanelet> intersected_lanelets = getIntersectedLanelets(convex_hull);
  // create local R-tree
  //std::vector<box, lanelet::Lanelet> rtree_nodes;
  //std::vector<std::pair<lanelet::BoundingBox2d, lanelet::Lanelet>> rtree_nodes;
  //for (const auto& lanelet : intersected_lanelets) {
  //    //const lanelet::BoundingBox2d lanelet_bbox2d = lanelet::geometry::boundingBox2d(lanelet);
  //    //point minPoint(lanelet_bbox2d.min().x(), lanelet_bbox2d.min().y());
  //    //point maxPoint(lanelet_bbox2d.max().x(), lanelet_bbox2d.max().y());
  //    //box bbox2d(minPoint, maxPoint);
  //    //rtree_nodes.emplace_back(std::make_pair(bbox2d, lanelet));
  //    auto bbox = lanelet::geometry::boundingBox2d(lanelet);
  //    rtree_nodes.emplace_back(bbox, lanelet);
  //}
  // create a local R-tree of intersected lanelets for later searching phase
  bgi::rtree<BoxAndLanelet, RtreeAlgo> local_rtree;
  //for (auto i = 0UL; i < intersected_lanelets.size(); ++i) {
  for (const auto& bbox_and_lanelet : intersected_lanelets) {
    //const auto box = boost::geometry::return_envelope<tier4_autoware_utils::Box2d>(geometries[i]);
    //const auto box = lanelet::geometry::boundingBox2d(intersected_lanelets[i]);
    local_rtree.insert(bbox_and_lanelet);
  }

  // Using R*-tree parameters
  //bgi::rtree<value, bgi::rstar<16>> local_rtree(rtree_nodes.begin(), rtree_nodes.end());


  // Step 3: Detailed Search
  /*
  for (const auto& bbox : boundingBoxes) {
      point minPoint(bbox.min().x(), bbox.min().y());
      point maxPoint(bbox.max().x(), bbox.max().y());
      box searchBox(minPoint, maxPoint);

      std::vector<value> results;
      local_rtree.query(bgi::intersects(searchBox), std::back_inserter(results));

      for (const auto& result : results) {
          const Lanelet& lanelet = result.second;
          if (geometry::intersects(lanelet, bbox)) {
              // Process the intersected lanelet
              std::cout << "Intersected Lanelet ID: " << lanelet.id() << std::endl;
          }
      }
  }
  */

  // filtering process
  for (size_t index = 0; index < transformed_objects.objects.size(); ++index) {
    const auto & transformed_object = transformed_objects.objects.at(index);
    const auto & input_object = input_msg->objects.at(index);
    //filterObject(transformed_object, input_object, intersected_lanelets, output_object_msg);
    filterObject(transformed_object, input_object, local_rtree, output_object_msg);
  }
  object_pub_->publish(output_object_msg);
  published_time_publisher_->publish_if_subscribed(object_pub_, output_object_msg.header.stamp);

  // Publish debug info
  const double pipeline_latency =
    std::chrono::duration<double, std::milli>(
      std::chrono::nanoseconds(
        (this->get_clock()->now() - output_object_msg.header.stamp).nanoseconds()))
      .count();
  debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
    "debug/pipeline_latency_ms", pipeline_latency);
}

bool ObjectLaneletFilterNode::filterObject(
  const autoware_perception_msgs::msg::DetectedObject & transformed_object,
  const autoware_perception_msgs::msg::DetectedObject & input_object,
  const bgi::rtree<BoxAndLanelet, RtreeAlgo> & local_rtree,
  //const lanelet::ConstLanelets & intersected_lanelets,
  autoware_perception_msgs::msg::DetectedObjects & output_object_msg)
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & label = transformed_object.classification.front().label;
  if (filter_target_.isTarget(label)) {
    bool filter_pass = true;
    // 1. is polygon overlap with road lanelets or shoulder lanelets
    if (filter_settings_.polygon_overlap_filter) {
      const bool is_polygon_overlap =
        //isObjectOverlapLanelets(transformed_object, intersected_lanelets);
        isObjectOverlapLanelets(transformed_object, local_rtree);
      filter_pass = filter_pass && is_polygon_overlap;
    }

    // 2. check if objects velocity is the same with the lanelet direction
    const bool orientation_not_available =
      transformed_object.kinematics.orientation_availability ==
      autoware_perception_msgs::msg::TrackedObjectKinematics::UNAVAILABLE;
    if (filter_settings_.lanelet_direction_filter && !orientation_not_available) {
      const bool is_same_direction =
        isSameDirectionWithLanelets(transformed_object, local_rtree);
      filter_pass = filter_pass && is_same_direction;
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

geometry_msgs::msg::Polygon ObjectLaneletFilterNode::setFootprint(
  const autoware_perception_msgs::msg::DetectedObject & detected_object)
{
  // no need, it is fast
  //autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);
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
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  MultiPoint2d candidate_points;
  for (const auto & object : detected_objects.objects) {
    const auto & pos = object.kinematics.pose_with_covariance.pose.position;
    const auto footprint = setFootprint(object);
    for (const auto & p : footprint.points) {
      // no need for
      //const geometry_msgs::msg::Point32 point_transformed =
      //  autoware::universe_utils::transformPoint(
      //    point, object.kinematics.pose_with_covariance.pose);
      // ?
      candidate_points.emplace_back(p.x + pos.x, p.y + pos.y);
    }
  }

  LinearRing2d convex_hull;
  bg::convex_hull(candidate_points, convex_hull);

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
//lanelet::ConstLanelets ObjectLaneletFilterNode::getIntersectedLanelets(
std::vector<BoxAndLanelet> ObjectLaneletFilterNode::getIntersectedLanelets(
  const LinearRing2d & convex_hull)
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  //lanelet::ConstLanelets intersected_lanelets;
  std::vector<BoxAndLanelet> intersected_lanelets_with_bbox;

  // convert convex_hull to a 2D bounding box for searching in the LaneletMap
  bg::model::box<bg::model::d2::point_xy<double>> bbox2d_convex_hull;
  bg::envelope(convex_hull, bbox2d_convex_hull);
  lanelet::BoundingBox2d bbox2d(
    lanelet::BasicPoint2d(
      bg::get<bg::min_corner, 0>(bbox2d_convex_hull),
      bg::get<bg::min_corner, 1>(bbox2d_convex_hull)),
    lanelet::BasicPoint2d(
      bg::get<bg::max_corner, 0>(bbox2d_convex_hull),
      bg::get<bg::max_corner, 1>(bbox2d_convex_hull)));

  lanelet::Lanelets candidate_lanelets = lanelet_map_ptr_->laneletLayer.search(bbox2d);
  for (const auto & lanelet : candidate_lanelets) {
    // only check the road lanelets and road shoulder lanelets
    if (
      lanelet.hasAttribute(lanelet::AttributeName::Subtype) &&
      (lanelet.attribute(lanelet::AttributeName::Subtype).value() ==
         lanelet::AttributeValueString::Road ||
       lanelet.attribute(lanelet::AttributeName::Subtype).value() == "road_shoulder")) {
      if (bg::intersects(convex_hull, lanelet.polygon2d().basicPolygon())) {
        //intersected_lanelets.emplace_back(lanelet);
        // create bbox using boost
        lanelet::BoundingBox2d lanelet_bbox = lanelet::geometry::boundingBox2d(lanelet);
        Point2d min_corner(lanelet_bbox.min().x(), lanelet_bbox.min().y());
        Point2d max_corner(lanelet_bbox.max().x(), lanelet_bbox.max().y());
        Box boost_bbox(min_corner, max_corner);

        intersected_lanelets_with_bbox.emplace_back(std::make_pair(boost_bbox, lanelet));
      }
    }
  }

  //return intersected_lanelets;
  return intersected_lanelets_with_bbox;
}

bool ObjectLaneletFilterNode::isObjectOverlapLanelets(
  const autoware_perception_msgs::msg::DetectedObject & object,
  //const lanelet::ConstLanelets & intersected_lanelets,
  const bgi::rtree<BoxAndLanelet, RtreeAlgo> & local_rtree)
{
  // if has bounding box, use polygon overlap
  if (utils::hasBoundingBox(object)) {
    autoware::universe_utils::ScopedTimeTrack st_1("bbox_filter", *time_keeper_);
    Polygon2d polygon;
    const auto footprint = setFootprint(object);
    for (const auto & point : footprint.points) {
      const geometry_msgs::msg::Point32 point_transformed =
        autoware::universe_utils::transformPoint(
          point, object.kinematics.pose_with_covariance.pose);
      polygon.outer().emplace_back(point_transformed.x, point_transformed.y);
    }
    polygon.outer().push_back(polygon.outer().front());
    //return isPolygonOverlapLanelets(polygon, intersected_lanelets);
    return isPolygonOverlapLaneletsRtree(polygon, local_rtree);
  } else {
    //autoware::universe_utils::ScopedTimeTrack st_2("no bbox filter: pre process", *time_keeper_);
    //LinearRing2d object_convex_hull = getConvexHullFromObjectFootprint(object)
    //// create bounding box to search in the rtree
    //std::vector<BoxAndLanelet> candidates;
    //bg::model::box<bg::model::d2::point_xy<double>> bbox;
    //bg::envelope(object_convex_hull, bbox);
    //local_rtree.query(bgi::intersects(bbox), std::back_inserter(candidates));
    //autoware::universe_utils::ScopedTimeTrack st_3("no bbox filter: main loop", *time_keeper_);
    //// if object do not have bounding box, check each footprint is inside polygon
    //for (const auto & point : object.shape.footprint.points) {
    //  const geometry_msgs::msg::Point32 point_transformed =
    //    autoware::universe_utils::transformPoint(
    //      point, object.kinematics.pose_with_covariance.pose);
    //  geometry_msgs::msg::Pose point2d;
    //  point2d.position.x = point_transformed.x;
    //  point2d.position.y = point_transformed.y;
    //  //for (const auto & lanelet : intersected_lanelets) {
    //  //  if (lanelet::utils::isInLanelet(point2d, lanelet, 0.0)) {
    //  //    return true;
    //  //  }
    //  //}
    //  for (const auto & candidate : candidates) {
    //    if (lanelet::utils::isInLanelet(point2d, candidate.second, 0.0)) {
    //      return true;
    //    }
    //  }
    //}

    //////////////////////////////////////////////////////////////////////////////////////////
    // might can use searching the point directly?
    for (const auto & point : object.shape.footprint.points) {
      const geometry_msgs::msg::Point32 point_transformed =
        autoware::universe_utils::transformPoint(
          point, object.kinematics.pose_with_covariance.pose);
      std::vector<BoxAndLanelet> candidates;
      Point2d p(point_transformed.x, point_transformed.y);
      local_rtree.query(bgi::nearest(p, 1), std::back_inserter(candidates));

      constexpr double eps = 1.0e-9;
      if (bg::distance(p, candidates[0].second.polygon2d().basicPolygon()) < eps) {
        return true;
      }
    }

    return false;
  }
}

bool ObjectLaneletFilterNode::isPolygonOverlapLaneletsRtree(
  const Polygon2d & polygon,
  const bgi::rtree<BoxAndLanelet, RtreeAlgo> & local_rtree)
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  std::vector<BoxAndLanelet> candidates;
  bg::model::box<bg::model::d2::point_xy<double>> bbox2d_convex_hull;
  bg::envelope(polygon, bbox2d_convex_hull);
  local_rtree.query(bgi::intersects(bbox2d_convex_hull), std::back_inserter(candidates));
  // directly calc polygon?

  //if (results.size == 0) {
  //  return false;
  //}

  for (const auto & box_and_lanelet : candidates) {
    //if (!boost::geometry::disjoint(polygon, box_and_lanelet.second.polygon2d().basicPolygon())) {
    if (bg::intersects(polygon, box_and_lanelet.second.polygon2d().basicPolygon())) {
      return true;
    }
  }

  return false;
}

bool ObjectLaneletFilterNode::isPolygonOverlapLanelets(
  const Polygon2d & polygon, const lanelet::ConstLanelets & intersected_lanelets)
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  for (const auto & lanelet : intersected_lanelets) {
    if (!bg::disjoint(polygon, lanelet.polygon2d().basicPolygon())) {
      return true;
    }
  }
  return false;
}


bool ObjectLaneletFilterNode::isSameDirectionWithLanelets(
  //const lanelet::ConstLanelets & lanelets,
  const autoware_perception_msgs::msg::DetectedObject & object,
  const bgi::rtree<BoxAndLanelet, RtreeAlgo> & local_rtree)
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);
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

  //std::vector<BoxAndLanelet> candidates;
  //local_rtree.query(bgi::within(Point2d(
  //  object.kinematics.pose_with_covariance.pose.position.x,
  //  object.kinematics.pose_with_covariance.pose.position.y)),
  //  std::back_inserter(candidates));
  constexpr double eps = 1.0e-9;
  std::vector<BoxAndLanelet> candidates;
  Point2d min_corner(object.kinematics.pose_with_covariance.pose.position.x-eps,
    object.kinematics.pose_with_covariance.pose.position.y-eps);
  Point2d max_corner(object.kinematics.pose_with_covariance.pose.position.x+eps,
    object.kinematics.pose_with_covariance.pose.position.y+eps);
  Box bbox(min_corner, max_corner);
  local_rtree.query(bgi::intersects(bbox), std::back_inserter(candidates));

  for (const auto & box_and_lanelet : candidates) {
    const bool is_in_lanelet =
      lanelet::utils::isInLanelet(object.kinematics.pose_with_covariance.pose,
        box_and_lanelet.second, 0.0);
    if (!is_in_lanelet) {
      continue;
    }

    const double lane_yaw = lanelet::utils::getLaneletAngle(
      box_and_lanelet.second, object.kinematics.pose_with_covariance.pose.position);
    const double delta_yaw = object_velocity_yaw - lane_yaw;
    const double normalized_delta_yaw = autoware::universe_utils::normalizeRadian(delta_yaw);
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
