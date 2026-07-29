#include "activity_control_pkg/route_target_publisher.hpp"

#include <angles/angles.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace activity_control_pkg
{

namespace
{
constexpr uint8_t kDropRouteChoice = 1;
constexpr uint8_t kLandingRouteChoice = 2;
constexpr uint8_t kDropFrameId = 0x11;
constexpr uint8_t kFlightSwitchFrameId = 0x44;
constexpr uint8_t kEnabledValue = 0x01;
constexpr uint8_t kDisabledValue = 0x00;
constexpr double kMonitorPeriodSec = 0.05;
}  // namespace

RouteTargetPublisherNode::RouteTargetPublisherNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("route_target_publisher", options),
  current_index_(0),
  route_choice_(0),
  state_(MissionState::WaitingRoute),
  returning_(false),
  has_height_(false),
  current_height_cm_(0.0),
  has_fine_data_(false),
  fine_error_x_px_(0),
  fine_error_y_px_(0),
  follow_timer_running_(false),
  landed_x_cm_(0.0),
  landed_y_cm_(0.0),
  landed_yaw_deg_(0.0),
  visual_active_(false),
  motion_hold_active_(true),
  landing_descent_active_(false),
  last_vision_fresh_(false)
{
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  robot_frame_ = declare_parameter<std::string>("robot_frame", "laser_link");
  position_tolerance_cm_ = declare_parameter<double>("position_tolerance_cm", 8.0);
  height_tolerance_cm_ = declare_parameter<double>("height_tolerance_cm", 8.0);
  yaw_tolerance_deg_ = declare_parameter<double>("yaw_tolerance_deg", 8.0);
  fine_data_timeout_sec_ = declare_parameter<double>("fine_data_timeout_sec", 0.2);
  follow_duration_sec_ = declare_parameter<double>("follow_duration_sec", 5.0);
  landed_hold_sec_ = declare_parameter<double>("landed_hold_sec", 5.0);
  landing_trigger_height_cm_ =
    declare_parameter<double>("landing_trigger_height_cm", 45.0);
  return_height_cm_ = declare_parameter<double>("return_height_cm", 150.0);
  declareRouteParameters(kDropRouteChoice);
  declareRouteParameters(kLandingRouteChoice);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto durable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_pub_ =
    create_publisher<std_msgs::msg::Float32MultiArray>("/target_position", durable_qos);
  visual_takeover_pub_ =
    create_publisher<std_msgs::msg::Bool>("/visual_takeover_active", durable_qos);
  motion_hold_pub_ =
    create_publisher<std_msgs::msg::Bool>("/motion_hold_active", durable_qos);
  landing_descent_pub_ =
    create_publisher<std_msgs::msg::Bool>("/landing_descent_active", durable_qos);
  vision_fresh_pub_ =
    create_publisher<std_msgs::msg::Bool>("/vision_fresh", durable_qos);
  mission_state_pub_ =
    create_publisher<std_msgs::msg::String>("/mission_state", durable_qos);
  waypoint_index_pub_ =
    create_publisher<std_msgs::msg::Int32>("/current_waypoint_index", durable_qos);
  route_choice_status_pub_ =
    create_publisher<std_msgs::msg::UInt8>("/route_choice_status", durable_qos);
  serial_command_pub_ =
    create_publisher<std_msgs::msg::UInt8MultiArray>("/serial_byte_command", 10);

  route_choice_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/route_choice", rclcpp::QoS(10).reliable(),
    std::bind(&RouteTargetPublisherNode::routeChoiceCallback, this, std::placeholders::_1));
  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/height", 10,
    std::bind(&RouteTargetPublisherNode::heightCallback, this, std::placeholders::_1));
  fine_data_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
    "/fine_data", 10,
    std::bind(&RouteTargetPublisherNode::fineDataCallback, this, std::placeholders::_1));
  serial_command_result_sub_ =
    create_subscription<std_msgs::msg::UInt8MultiArray>(
    "/serial_byte_command_result", 10,
    std::bind(
      &RouteTargetPublisherNode::serialCommandResultCallback,
      this, std::placeholders::_1));

  last_fine_data_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  follow_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  state_start_time_ = now();
  monitor_timer_ = create_wall_timer(
    std::chrono::duration<double>(kMonitorPeriodSec),
    std::bind(&RouteTargetPublisherNode::monitorTimerCallback, this));

  publishVisualState(false);
  publishLandingDescentState(false);
  publishMotionHold(true);
  publishVisionFresh(false);
  publishMissionState();
  publishCurrentWaypointIndex();

  RCLCPP_INFO(
    get_logger(),
    "D-task route controller ready. Waiting for /route_choice: 1=drop, 2=landing.");
  RCLCPP_INFO(
    get_logger(),
    "fine_data_timeout=%.3fs follow=%.1fs landing_trigger=%.1fcm landed_hold=%.1fs",
    fine_data_timeout_sec_, follow_duration_sec_,
    landing_trigger_height_cm_, landed_hold_sec_);
}

void RouteTargetPublisherNode::declareRouteParameters(uint8_t route_choice)
{
  const std::string prefix = "route_" + std::to_string(route_choice) + "_";
  declare_parameter<std::vector<double>>(
    prefix + "x_cm", std::vector<double>{});
  declare_parameter<std::vector<double>>(
    prefix + "y_cm", std::vector<double>{});
  declare_parameter<std::vector<double>>(
    prefix + "z_cm", std::vector<double>{});
  declare_parameter<std::vector<double>>(
    prefix + "yaw_deg", std::vector<double>{});
  declare_parameter<std::vector<int64_t>>(
    prefix + "type", std::vector<int64_t>{});
}

std::vector<Target> RouteTargetPublisherNode::loadConfiguredRoute(uint8_t route_choice) const
{
  const std::string prefix = "route_" + std::to_string(route_choice) + "_";
  const auto x = get_parameter(prefix + "x_cm").as_double_array();
  const auto y = get_parameter(prefix + "y_cm").as_double_array();
  const auto z = get_parameter(prefix + "z_cm").as_double_array();
  const auto yaw = get_parameter(prefix + "yaw_deg").as_double_array();
  const auto type = get_parameter(prefix + "type").as_integer_array();

  if (x.size() != y.size() || x.size() != z.size() ||
    x.size() != yaw.size() || x.size() != type.size())
  {
    throw std::runtime_error("route parameter arrays must have equal lengths");
  }

  std::vector<Target> route;
  route.reserve(x.size());
  for (std::size_t index = 0; index < x.size(); ++index) {
    route.push_back(Target{
      x[index], y[index], z[index], yaw[index],
      static_cast<WaypointType>(type[index])});
  }
  validateRoute(route, route_choice);
  return route;
}

void RouteTargetPublisherNode::validateRoute(
  const std::vector<Target> & route, uint8_t route_choice) const
{
  if (route.empty()) {
    throw std::runtime_error("selected route is empty");
  }
  const WaypointType expected_search =
    route_choice == kDropRouteChoice ? WaypointType::SearchDrop : WaypointType::SearchLand;
  bool has_expected_search = false;
  for (const auto & target : route) {
    if (target.type != WaypointType::Normal &&
      target.type != WaypointType::SearchDrop &&
      target.type != WaypointType::SearchLand)
    {
      throw std::runtime_error("waypoint type must be 1, 2, or 3");
    }
    if (target.type == expected_search) {
      has_expected_search = true;
    }
    if (target.type != WaypointType::Normal && target.type != expected_search) {
      throw std::runtime_error("route contains a search waypoint for the other mission");
    }
  }
  if (!has_expected_search) {
    throw std::runtime_error("selected route has no task search waypoint");
  }
}

void RouteTargetPublisherNode::routeChoiceCallback(
  const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (msg->data != kDropRouteChoice && msg->data != kLandingRouteChoice) {
    RCLCPP_WARN(
      get_logger(), "Ignoring invalid /route_choice=%u; valid values are 1 and 2.",
      static_cast<unsigned>(msg->data));
    return;
  }
  if (state_ != MissionState::WaitingRoute && state_ != MissionState::Completed) {
    RCLCPP_WARN(
      get_logger(), "Ignoring /route_choice=%u because a mission is already active.",
      static_cast<unsigned>(msg->data));
    return;
  }
  try {
    loadRoute(msg->data);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      get_logger(), "Cannot load route %u: %s",
      static_cast<unsigned>(msg->data), error.what());
  }
}

void RouteTargetPublisherNode::loadRoute(uint8_t route_choice)
{
  targets_ = loadConfiguredRoute(route_choice);
  current_index_ = 0;
  route_choice_ = route_choice;
  returning_ = false;
  has_fine_data_ = false;
  follow_timer_running_ = false;
  publishVisualState(false);
  publishLandingDescentState(false);
  publishMotionHold(false);
  setState(MissionState::Navigating);

  std_msgs::msg::UInt8 route_msg;
  route_msg.data = route_choice_;
  route_choice_status_pub_->publish(route_msg);
  publishCurrentTarget();
  publishCurrentWaypointIndex();
  RCLCPP_INFO(
    get_logger(), "Loaded route %u with %zu waypoints.",
    static_cast<unsigned>(route_choice_), targets_.size());
}

void RouteTargetPublisherNode::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  current_height_cm_ = static_cast<double>(msg->data);
  has_height_ = true;
}

void RouteTargetPublisherNode::fineDataCallback(
  const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "/fine_data requires [forward_error_px, lateral_error_px].");
    return;
  }
  fine_error_x_px_ = msg->data[0];
  fine_error_y_px_ = msg->data[1];
  has_fine_data_ = true;
  last_fine_data_time_ = now();
}

void RouteTargetPublisherNode::serialCommandResultCallback(
  const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  if (msg->data.size() != 3) {
    RCLCPP_WARN(
      get_logger(),
      "/serial_byte_command_result must contain [frame_id, value, success].");
    return;
  }

  const uint8_t frame_id = msg->data[0];
  const uint8_t value = msg->data[1];
  const bool success = msg->data[2] != 0;
  bool expected_result = false;

  if (state_ == MissionState::DropCommandPending) {
    expected_result = frame_id == kDropFrameId && value == kEnabledValue;
  } else if (state_ == MissionState::LandingStopPending) {
    expected_result = frame_id == kFlightSwitchFrameId && value == kEnabledValue;
  } else if (state_ == MissionState::TakeoffCommandPending) {
    expected_result = frame_id == kFlightSwitchFrameId && value == kDisabledValue;
  }
  if (!expected_result) {
    return;
  }

  if (!success) {
    publishMotionHold(true);
    publishVisualState(false);
    publishLandingDescentState(false);
    setState(MissionState::Error);
    RCLCPP_ERROR(
      get_logger(),
      "Serial frame 0x%02X:0x%02X failed; mission is locked in ERROR.",
      static_cast<unsigned>(frame_id), static_cast<unsigned>(value));
    return;
  }

  if (state_ == MissionState::DropCommandPending) {
    startReturnRoute(false);
  } else if (state_ == MissionState::LandingStopPending) {
    setState(MissionState::LandedHold);
  } else if (state_ == MissionState::TakeoffCommandPending) {
    startReturnRoute(true);
  }
}

bool RouteTargetPublisherNode::hasFreshFineData(const rclcpp::Time & now_time) const
{
  return has_fine_data_ &&
         last_fine_data_time_.nanoseconds() != 0 &&
         (now_time - last_fine_data_time_).seconds() <= fine_data_timeout_sec_;
}

void RouteTargetPublisherNode::monitorTimerCallback()
{
  const rclcpp::Time now_time = now();
  const bool vision_fresh = hasFreshFineData(now_time);
  if (vision_fresh != last_vision_fresh_) {
    publishVisionFresh(vision_fresh);
    last_vision_fresh_ = vision_fresh;
  }

  if (state_ == MissionState::WaitingRoute ||
    state_ == MissionState::Completed ||
    state_ == MissionState::Error)
  {
    return;
  }

  if (state_ == MissionState::DropCommandPending ||
    state_ == MissionState::LandingStopPending ||
    state_ == MissionState::TakeoffCommandPending)
  {
    publishMotionHold(true);
    return;
  }

  if (state_ == MissionState::LandedHold) {
    if ((now_time - state_start_time_).seconds() >= landed_hold_sec_) {
      setState(MissionState::TakeoffCommandPending);
      publishSerialByteCommand(kFlightSwitchFrameId, kDisabledValue);
    }
    return;
  }

  double x_cm = 0.0;
  double y_cm = 0.0;
  double yaw_deg = 0.0;
  if (!getCurrentPose(x_cm, y_cm, yaw_deg)) {
    publishMotionHold(true);
    return;
  }

  if (state_ == MissionState::FollowDrop) {
    if (!vision_fresh) {
      publishMotionHold(true);
      follow_timer_running_ = false;
      return;
    }
    publishMotionHold(false);
    if (!follow_timer_running_) {
      follow_start_time_ = now_time;
      follow_timer_running_ = true;
      RCLCPP_INFO(get_logger(), "Fresh car tracking acquired; starting continuous 5s timer.");
    }
    if ((now_time - follow_start_time_).seconds() >= follow_duration_sec_) {
      publishMotionHold(true);
      publishVisualState(false);
      setState(MissionState::DropCommandPending);
      publishSerialByteCommand(kDropFrameId, kEnabledValue);
    }
    return;
  }

  if (state_ == MissionState::FollowLand) {
    if (!vision_fresh) {
      publishMotionHold(true);
      return;
    }
    publishMotionHold(false);
    if (has_height_ && current_height_cm_ < landing_trigger_height_cm_) {
      landed_x_cm_ = x_cm;
      landed_y_cm_ = y_cm;
      landed_yaw_deg_ = yaw_deg;
      publishMotionHold(true);
      publishVisualState(false);
      publishLandingDescentState(false);
      setState(MissionState::LandingStopPending);
      publishSerialByteCommand(kFlightSwitchFrameId, kEnabledValue);
    }
    return;
  }

  if (current_index_ >= targets_.size()) {
    if (returning_) {
      completeMission();
    } else {
      startReturnRoute(false);
    }
    return;
  }

  const Target & target = targets_[current_index_];
  const bool is_search =
    target.type == WaypointType::SearchDrop || target.type == WaypointType::SearchLand;
  if (is_search && vision_fresh && !returning_) {
    beginSearchTask(target, now_time);
    return;
  }

  publishMotionHold(false);
  if (isCurrentTargetReached(x_cm, y_cm, yaw_deg)) {
    RCLCPP_INFO(
      get_logger(), "Reached waypoint %zu/%zu type=%d.",
      current_index_ + 1, targets_.size(), static_cast<int>(target.type));
    advanceTarget();
  }
}

void RouteTargetPublisherNode::beginSearchTask(
  const Target & target, const rclcpp::Time & now_time)
{
  targets_.resize(current_index_ + 1);
  follow_timer_running_ = false;
  publishVisualState(true);
  publishMotionHold(false);

  Target visual_hold = target;
  if (target.type == WaypointType::SearchLand) {
    visual_hold.z_cm = 0.0;
    publishLandingDescentState(true);
    setState(MissionState::FollowLand);
  } else {
    publishLandingDescentState(false);
    follow_start_time_ = now_time;
    follow_timer_running_ = true;
    setState(MissionState::FollowDrop);
  }
  publishTarget(visual_hold);
  RCLCPP_INFO(
    get_logger(),
    "Car detected at search waypoint %zu; removed later search waypoints and entered %s.",
    current_index_, stateName(state_));
}

void RouteTargetPublisherNode::startReturnRoute(bool takeoff_from_car)
{
  targets_.clear();
  if (takeoff_from_car) {
    targets_.push_back(Target{
      landed_x_cm_, landed_y_cm_, return_height_cm_, landed_yaw_deg_,
      WaypointType::Normal});
  }
  targets_.push_back(Target{
    0.0, 0.0, return_height_cm_, 0.0, WaypointType::Normal});
  targets_.push_back(Target{0.0, 0.0, 0.0, 0.0, WaypointType::Normal});
  current_index_ = 0;
  returning_ = true;
  follow_timer_running_ = false;
  publishVisualState(false);
  publishLandingDescentState(false);
  publishMotionHold(false);
  setState(MissionState::Returning);
  publishCurrentWaypointIndex();
  publishCurrentTarget();
}

void RouteTargetPublisherNode::advanceTarget()
{
  ++current_index_;
  publishCurrentWaypointIndex();
  if (current_index_ >= targets_.size()) {
    if (returning_) {
      completeMission();
    } else {
      startReturnRoute(false);
    }
    return;
  }
  publishCurrentTarget();
}

void RouteTargetPublisherNode::completeMission()
{
  publishVisualState(false);
  publishLandingDescentState(false);
  publishMotionHold(true);
  setState(MissionState::Completed);
  RCLCPP_INFO(get_logger(), "Mission complete; target velocity is held at zero.");
}

bool RouteTargetPublisherNode::getCurrentPose(
  double & x_cm, double & y_cm, double & yaw_deg)
{
  try {
    const auto transform =
      tf_buffer_->lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero);
    x_cm = transform.transform.translation.x * 100.0;
    y_cm = transform.transform.translation.y * 100.0;
    tf2::Quaternion quaternion;
    tf2::fromMsg(transform.transform.rotation, quaternion);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    yaw_deg = yaw * 180.0 / M_PI;
    return true;
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "TF %s -> %s unavailable: %s",
      map_frame_.c_str(), robot_frame_.c_str(), error.what());
    return false;
  }
}

bool RouteTargetPublisherNode::isCurrentTargetReached(
  double x_cm, double y_cm, double yaw_deg) const
{
  if (current_index_ >= targets_.size() || !has_height_) {
    return false;
  }
  const auto & target = targets_[current_index_];
  const double distance_xy = std::hypot(target.x_cm - x_cm, target.y_cm - y_cm);
  const double height_error = std::fabs(target.z_cm - current_height_cm_);
  const double yaw_error = std::fabs(normalizeAngleDeg(target.yaw_deg - yaw_deg));
  return distance_xy <= position_tolerance_cm_ &&
         height_error <= height_tolerance_cm_ &&
         yaw_error <= yaw_tolerance_deg_;
}

void RouteTargetPublisherNode::publishCurrentTarget()
{
  if (current_index_ < targets_.size()) {
    publishTarget(targets_[current_index_]);
  }
}

void RouteTargetPublisherNode::publishTarget(const Target & target)
{
  std_msgs::msg::Float32MultiArray msg;
  msg.data = {
    static_cast<float>(target.x_cm),
    static_cast<float>(target.y_cm),
    static_cast<float>(target.z_cm),
    static_cast<float>(target.yaw_deg)};
  target_pub_->publish(msg);
  RCLCPP_INFO(
    get_logger(), "Target: (%.1f, %.1f, %.1f, %.1f), type=%d",
    target.x_cm, target.y_cm, target.z_cm, target.yaw_deg,
    static_cast<int>(target.type));
}

void RouteTargetPublisherNode::publishVisualState(bool active)
{
  visual_active_ = active;
  std_msgs::msg::Bool msg;
  msg.data = active;
  visual_takeover_pub_->publish(msg);
}

void RouteTargetPublisherNode::publishMotionHold(bool active)
{
  if (motion_hold_active_ == active && state_ != MissionState::WaitingRoute) {
    return;
  }
  motion_hold_active_ = active;
  std_msgs::msg::Bool msg;
  msg.data = active;
  motion_hold_pub_->publish(msg);
}

void RouteTargetPublisherNode::publishLandingDescentState(bool active)
{
  landing_descent_active_ = active;
  std_msgs::msg::Bool msg;
  msg.data = active;
  landing_descent_pub_->publish(msg);
}

void RouteTargetPublisherNode::publishVisionFresh(bool fresh)
{
  std_msgs::msg::Bool msg;
  msg.data = fresh;
  vision_fresh_pub_->publish(msg);
}

void RouteTargetPublisherNode::publishCurrentWaypointIndex()
{
  std_msgs::msg::Int32 msg;
  msg.data = current_index_ < targets_.size() ?
    static_cast<int32_t>(current_index_) : -1;
  waypoint_index_pub_->publish(msg);
}

void RouteTargetPublisherNode::publishSerialByteCommand(uint8_t frame_id, uint8_t value)
{
  std_msgs::msg::UInt8MultiArray msg;
  msg.data = {frame_id, value};
  serial_command_pub_->publish(msg);
  RCLCPP_INFO(
    get_logger(), "Requested serial frame id=0x%02X data=0x%02X.",
    static_cast<unsigned>(frame_id), static_cast<unsigned>(value));
}

void RouteTargetPublisherNode::setState(MissionState state)
{
  if (state_ != state) {
    RCLCPP_INFO(
      get_logger(), "Mission state: %s -> %s",
      stateName(state_), stateName(state));
  }
  state_ = state;
  state_start_time_ = now();
  publishMissionState();
}

void RouteTargetPublisherNode::publishMissionState()
{
  std_msgs::msg::String msg;
  msg.data = stateName(state_);
  mission_state_pub_->publish(msg);
}

const char * RouteTargetPublisherNode::stateName(MissionState state)
{
  switch (state) {
    case MissionState::WaitingRoute: return "WAITING_ROUTE";
    case MissionState::Navigating: return "NAVIGATING";
    case MissionState::FollowDrop: return "FOLLOW_DROP";
    case MissionState::FollowLand: return "FOLLOW_LAND";
    case MissionState::DropCommandPending: return "DROP_COMMAND_PENDING";
    case MissionState::LandingStopPending: return "LANDING_STOP_PENDING";
    case MissionState::LandedHold: return "LANDED_HOLD";
    case MissionState::TakeoffCommandPending: return "TAKEOFF_COMMAND_PENDING";
    case MissionState::Returning: return "RETURNING";
    case MissionState::Completed: return "COMPLETED";
    case MissionState::Error: return "ERROR";
  }
  return "UNKNOWN";
}

double RouteTargetPublisherNode::normalizeAngleDeg(double angle_deg)
{
  return angles::to_degrees(angles::normalize_angle(angles::from_degrees(angle_deg)));
}

}  // namespace activity_control_pkg
