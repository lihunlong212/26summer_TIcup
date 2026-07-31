#include "activity_control_pkg/route_target_publisher.hpp"

#include <angles/angles.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
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
constexpr uint8_t kDropFlyChoice = 1;
constexpr uint8_t kLandingFlyChoice = 2;
constexpr uint8_t kDropFrameId = 0x11;
constexpr uint8_t kFlightSwitchFrameId = 0x44;
constexpr uint8_t kEnabledValue = 0x01;
constexpr uint8_t kDisabledValue = 0x00;
constexpr double kMonitorPeriodSec = 0.05;
}  // namespace

RouteTargetPublisherNode::RouteTargetPublisherNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("route_target_publisher", options),
  current_index_(0),
  fly_choice_(0),
  state_(MissionState::WaitingRoute),
  returning_(false),
  search_segment_active_(false),
  has_height_(false),
  current_height_cm_(0.0),
  has_fine_data_(false),
  fine_error_x_px_(0),
  fine_error_y_px_(0),
  drop_aligned_frame_count_(0),
  task_x_cm_(0.0),
  task_y_cm_(0.0),
  task_yaw_deg_(0.0),
  visual_active_(false),
  motion_hold_active_(true),
  visual_descent_active_(false),
  last_vision_fresh_(false),
  drone_state_enabled_(false),
  has_published_drone_state_(false),
  last_drone_state_(0)
{
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  robot_frame_ = declare_parameter<std::string>("robot_frame", "laser_link");
  position_tolerance_cm_ = declare_parameter<double>("position_tolerance_cm", 8.0);
  height_tolerance_cm_ = declare_parameter<double>("height_tolerance_cm", 8.0);
  yaw_tolerance_deg_ = declare_parameter<double>("yaw_tolerance_deg", 8.0);
  takeoff_hover_sec_ = declare_parameter<double>("takeoff_hover_sec", 2.0);
  fine_data_timeout_sec_ = declare_parameter<double>("fine_data_timeout_sec", 0.2);
  pre_descent_alignment_sec_ =
    declare_parameter<double>("pre_descent_alignment_sec", 2.0);
  drop_target_height_cm_ =
    declare_parameter<double>("drop_target_height_cm", 50.0);
  drop_trigger_height_cm_ =
    declare_parameter<double>("drop_trigger_height_cm", 57.0);
  drop_alignment_tolerance_px_ =
    declare_parameter<double>("drop_alignment_tolerance_px", 100.0);
  drop_alignment_required_frames_ =
    declare_parameter<int64_t>("drop_alignment_required_frames", 3);
  landed_hold_sec_ = declare_parameter<double>("landed_hold_sec", 5.0);
  landing_trigger_height_cm_ =
    declare_parameter<double>("landing_trigger_height_cm", 45.0);
  drone_state_action_height_cm_ =
    declare_parameter<double>("drone_state_action_height_cm", 80.0);
  final_landing_stop_height_cm_ =
    declare_parameter<double>("final_landing_stop_height_cm", 21.0);
  declare_parameter<std::vector<std::string>>(
    "post_task_return_waypoints",
    std::vector<std::string>{"(0 0 150 0)", "(0 0 0 0)"});
  if (takeoff_hover_sec_ < 0.0 ||
    fine_data_timeout_sec_ <= 0.0 ||
    pre_descent_alignment_sec_ < 0.0 ||
    drop_target_height_cm_ < 0.0 ||
    drop_trigger_height_cm_ < drop_target_height_cm_ ||
    drop_alignment_tolerance_px_ < 0.0 ||
    drop_alignment_required_frames_ <= 0 ||
    drone_state_action_height_cm_ < 0.0 ||
    final_landing_stop_height_cm_ <= 0.0)
  {
    throw std::invalid_argument(
            "takeoff hover must be non-negative; fine_data_timeout_sec must be "
            "positive; pre-descent alignment and "
            "drop alignment parameters must be non-negative; "
            "drop_trigger_height_cm must be greater than or equal to "
            "drop_target_height_cm; "
            "and drone_state_action_height_cm must use non-negative values; "
            "final_landing_stop_height_cm must be positive; "
            "drop frame count must be positive");
  }
  declareRouteParameters(kDropFlyChoice);
  declareRouteParameters(kLandingFlyChoice);
  post_task_return_waypoints_ = loadPostTaskReturnWaypoints();
  if (post_task_return_waypoints_.back().z_cm >= final_landing_stop_height_cm_) {
    throw std::invalid_argument(
            "the final post_task_return_waypoint height must be lower than "
            "final_landing_stop_height_cm");
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto durable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_pub_ =
    create_publisher<std_msgs::msg::Float32MultiArray>("/target_position", durable_qos);
  visual_takeover_pub_ =
    create_publisher<std_msgs::msg::Bool>("/visual_takeover_active", durable_qos);
  motion_hold_pub_ =
    create_publisher<std_msgs::msg::Bool>("/motion_hold_active", durable_qos);
  visual_descent_pub_ =
    create_publisher<std_msgs::msg::Bool>("/visual_descent_active", durable_qos);
  vision_fresh_pub_ =
    create_publisher<std_msgs::msg::Bool>("/vision_fresh", durable_qos);
  drone_state_pub_ =
    create_publisher<std_msgs::msg::UInt8>("/drone_state", durable_qos);
  waypoint_index_pub_ =
    create_publisher<std_msgs::msg::Int32>("/current_waypoint_index", durable_qos);
  fly_choice_status_pub_ =
    create_publisher<std_msgs::msg::UInt8>("/fly_choice_status", durable_qos);
  serial_command_pub_ =
    create_publisher<std_msgs::msg::UInt8MultiArray>("/serial_byte_command", 10);

  fly_choice_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/fly_choice", rclcpp::QoS(10).reliable(),
    std::bind(&RouteTargetPublisherNode::flyChoiceCallback, this, std::placeholders::_1));
  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/height", 10,
    std::bind(&RouteTargetPublisherNode::heightCallback, this, std::placeholders::_1));
  fine_data_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
    "/fine_data", 10,
    std::bind(&RouteTargetPublisherNode::fineDataCallback, this, std::placeholders::_1));
  target_velocity_sub_ =
    create_subscription<std_msgs::msg::Float32MultiArray>(
    "/target_velocity", 10,
    std::bind(
      &RouteTargetPublisherNode::targetVelocityCallback,
      this, std::placeholders::_1));
  serial_command_result_sub_ =
    create_subscription<std_msgs::msg::UInt8MultiArray>(
    "/serial_byte_command_result", 10,
    std::bind(
      &RouteTargetPublisherNode::serialCommandResultCallback,
      this, std::placeholders::_1));

  last_fine_data_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  state_start_time_ = now();
  monitor_timer_ = create_wall_timer(
    std::chrono::duration<double>(kMonitorPeriodSec),
    std::bind(&RouteTargetPublisherNode::monitorTimerCallback, this));

  publishVisualState(false);
  publishVisualDescentState(false);
  publishMotionHold(true);
  publishVisionFresh(false);
  publishCurrentWaypointIndex();

  RCLCPP_INFO(
    get_logger(),
    "D-task route controller ready. Waiting for /fly_choice: 1=drop, 2=landing.");
  RCLCPP_INFO(
    get_logger(),
    "takeoff_hover=%.1fs fine_data_timeout=%.3fs pre_descent_alignment=%.1fs "
    "drop_target=%.1fcm drop_trigger=%.1fcm/%.1fpx/%ldframes "
    "landing_trigger=%.1fcm final_stop=%.1fcm "
    "state_action_height=%.1fcm landed_hold=%.1fs",
    takeoff_hover_sec_, fine_data_timeout_sec_, pre_descent_alignment_sec_,
    drop_target_height_cm_, drop_trigger_height_cm_,
    drop_alignment_tolerance_px_, drop_alignment_required_frames_,
    landing_trigger_height_cm_, final_landing_stop_height_cm_,
    drone_state_action_height_cm_, landed_hold_sec_);
}

void RouteTargetPublisherNode::declareRouteParameters(uint8_t fly_choice)
{
  const std::string prefix =
    fly_choice == kDropFlyChoice ? "route_drop_" : "route_land_";
  declare_parameter<std::vector<std::string>>(
    prefix + "waypoints", std::vector<std::string>{});
  declare_parameter<int64_t>(prefix + "normal_count", 0);
}

std::vector<Target> RouteTargetPublisherNode::loadConfiguredRoute(uint8_t fly_choice) const
{
  const std::string prefix =
    fly_choice == kDropFlyChoice ? "route_drop_" : "route_land_";
  const auto waypoint_texts = get_parameter(prefix + "waypoints").as_string_array();
  const int64_t normal_count = get_parameter(prefix + "normal_count").as_int();
  if (waypoint_texts.empty()) {
    throw std::runtime_error("selected route is empty");
  }
  if (normal_count < 0 ||
    static_cast<std::size_t>(normal_count) >= waypoint_texts.size())
  {
    throw std::runtime_error(
            prefix + "normal_count must be >= 0 and smaller than waypoint count");
  }

  const WaypointType search_type =
    fly_choice == kDropFlyChoice ?
    WaypointType::SearchDrop : WaypointType::SearchLand;
  std::vector<Target> route;
  route.reserve(waypoint_texts.size());
  for (std::size_t index = 0; index < waypoint_texts.size(); ++index) {
    const WaypointType type =
      index < static_cast<std::size_t>(normal_count) ?
      WaypointType::Normal : search_type;
    route.push_back(parseWaypoint(waypoint_texts[index], type));
  }
  return route;
}

std::vector<Target> RouteTargetPublisherNode::loadPostTaskReturnWaypoints() const
{
  const auto waypoint_texts =
    get_parameter("post_task_return_waypoints").as_string_array();
  if (waypoint_texts.empty()) {
    throw std::runtime_error("post_task_return_waypoints must not be empty");
  }

  std::vector<Target> route;
  route.reserve(waypoint_texts.size());
  for (const auto & waypoint_text : waypoint_texts) {
    route.push_back(parseWaypoint(waypoint_text, WaypointType::Normal));
  }
  return route;
}

void RouteTargetPublisherNode::flyChoiceCallback(
  const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (msg->data != kDropFlyChoice && msg->data != kLandingFlyChoice) {
    RCLCPP_WARN(
      get_logger(), "Ignoring invalid /fly_choice=%u; valid values are 1 and 2.",
      static_cast<unsigned>(msg->data));
    return;
  }
  if (state_ != MissionState::WaitingRoute && state_ != MissionState::Completed) {
    RCLCPP_WARN(
      get_logger(), "Ignoring /fly_choice=%u because a mission is already active.",
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

void RouteTargetPublisherNode::loadRoute(uint8_t fly_choice)
{
  targets_ = loadConfiguredRoute(fly_choice);
  current_index_ = 0;
  fly_choice_ = fly_choice;
  returning_ = false;
  search_segment_active_ = false;
  has_fine_data_ = false;
  drone_state_enabled_ = false;
  resetDropAlignmentCount();
  publishVisualState(false);
  publishVisualDescentState(false);
  setState(MissionState::Navigating);

  std_msgs::msg::UInt8 choice_msg;
  choice_msg.data = fly_choice_;
  fly_choice_status_pub_->publish(choice_msg);
  updateSearchSegmentState();
  publishCurrentTarget();
  publishCurrentWaypointIndex();
  publishMotionHold(false);
  RCLCPP_INFO(
    get_logger(), "Loaded fly choice %u with %zu waypoints.",
    static_cast<unsigned>(fly_choice_), targets_.size());
}

void RouteTargetPublisherNode::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  current_height_cm_ = static_cast<double>(msg->data);
  has_height_ = true;
  publishDroneStateIfReady();
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
  if (!search_segment_active_ && !visual_active_) {
    return;
  }
  const rclcpp::Time received_time = now();
  const bool frame_gap_timed_out =
    has_fine_data_ &&
    last_fine_data_time_.nanoseconds() != 0 &&
    (received_time - last_fine_data_time_).seconds() > fine_data_timeout_sec_;
  if (state_ == MissionState::FollowDrop && frame_gap_timed_out) {
    resetDropAlignmentCount();
  }
  fine_error_x_px_ = msg->data[0];
  fine_error_y_px_ = msg->data[1];
  has_fine_data_ = true;
  last_fine_data_time_ = received_time;

  // A frame is counted exactly once here, never once per control/monitor cycle.
  if (state_ == MissionState::FollowDrop) {
    if (has_height_ &&
      current_height_cm_ <= drop_trigger_height_cm_ &&
      isFineDataAligned())
    {
      drop_aligned_frame_count_ = std::min(
        drop_aligned_frame_count_ + 1, drop_alignment_required_frames_);
      RCLCPP_INFO(
        get_logger(), "Drop alignment frame %ld/%ld: error=(%d,%d)px.",
        drop_aligned_frame_count_, drop_alignment_required_frames_,
        fine_error_x_px_, fine_error_y_px_);
    } else {
      resetDropAlignmentCount();
    }
  }
}

void RouteTargetPublisherNode::targetVelocityCallback(
  const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() != 4 ||
    !std::all_of(
      msg->data.begin(), msg->data.end(),
      [](float value) {return std::isfinite(value);}))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Ignoring invalid /target_velocity; expected exactly four finite values.");
    return;
  }
  if (state_ == MissionState::WaitingRoute || state_ == MissionState::Error) {
    return;
  }
  if (!drone_state_enabled_) {
    drone_state_enabled_ = true;
    RCLCPP_INFO(
      get_logger(),
      "First valid target velocity observed; /drone_state reporting is enabled.");
    publishDroneStateIfReady(true);
  }
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
  } else if (state_ == MissionState::LandingStopPending ||
    state_ == MissionState::FinalLandingStopPending)
  {
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
    publishVisualDescentState(false);
    setState(MissionState::Error);
    RCLCPP_ERROR(
      get_logger(),
      "Serial frame 0x%02X:0x%02X failed; mission is locked in ERROR.",
      static_cast<unsigned>(frame_id), static_cast<unsigned>(value));
    return;
  }

  if (state_ == MissionState::DropCommandPending) {
    startReturnRoute(true);
  } else if (state_ == MissionState::LandingStopPending) {
    setState(MissionState::LandedHold);
  } else if (state_ == MissionState::FinalLandingStopPending) {
    completeMission();
  } else if (state_ == MissionState::TakeoffCommandPending) {
    // The car may have moved during the five-second hold. Keep publishing zero
    // velocity until a fresh TF pose can be captured after takeoff is enabled.
    setState(MissionState::WaitingTakeoffPose);
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
  const bool vision_fresh =
    (search_segment_active_ || visual_active_) && hasFreshFineData(now_time);
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
    state_ == MissionState::FinalLandingStopPending ||
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

  if ((state_ == MissionState::HighAlignLand ||
    state_ == MissionState::FollowLand) &&
    has_height_ && current_height_cm_ <= landing_trigger_height_cm_)
  {
    search_segment_active_ = false;
    has_fine_data_ = false;
    publishMotionHold(true);
    publishVisualState(false);
    publishVisualDescentState(false);
    publishVisionFresh(false);
    last_vision_fresh_ = false;
    setState(MissionState::LandingStopPending);
    publishSerialByteCommand(kFlightSwitchFrameId, kEnabledValue);
    return;
  }

  if (state_ == MissionState::Navigating &&
    !returning_ &&
    fly_choice_ == kDropFlyChoice &&
    current_index_ == 0 &&
    current_index_ < targets_.size() &&
    takeoff_hover_sec_ > 0.0 &&
    has_height_ &&
    std::fabs(current_height_cm_ - targets_[current_index_].z_cm) <=
    height_tolerance_cm_)
  {
    setState(MissionState::TakeoffHover);
    RCLCPP_INFO(
      get_logger(),
      "Drop mission reached the initial flight-altitude band; starting %.1f s "
      "countdown without waiting for XY/yaw stability.",
      takeoff_hover_sec_);
  }

  if (state_ == MissionState::TakeoffHover) {
    publishMotionHold(false);
    if ((now_time - state_start_time_).seconds() >= takeoff_hover_sec_) {
      RCLCPP_INFO(
        get_logger(),
        "Initial flight-altitude countdown completed after %.1f s.",
        takeoff_hover_sec_);
      setState(MissionState::Navigating);
      advanceTarget();
    }
    return;
  }

  if (isFinalReturnLeg() &&
    has_height_ &&
    current_height_cm_ < final_landing_stop_height_cm_)
  {
    RCLCPP_INFO(
      get_logger(),
      "Final return descent reached %.1f cm (< %.1f cm); forcing continuous "
      "[0,0,0,0] target velocity and requesting flight lock.",
      current_height_cm_, final_landing_stop_height_cm_);
    publishMotionHold(true);
    setState(MissionState::FinalLandingStopPending);
    publishSerialByteCommand(kFlightSwitchFrameId, kEnabledValue);
    return;
  }

  double x_cm = 0.0;
  double y_cm = 0.0;
  double yaw_deg = 0.0;
  if (!getCurrentPose(x_cm, y_cm, yaw_deg)) {
    if (state_ == MissionState::HighAlignDrop ||
      state_ == MissionState::HighAlignLand)
    {
      state_start_time_ = now_time;
    }
    publishMotionHold(true);
    return;
  }

  if (state_ == MissionState::WaitingTakeoffPose) {
    task_x_cm_ = x_cm;
    task_y_cm_ = y_cm;
    task_yaw_deg_ = yaw_deg;
    startReturnRoute(true);
    return;
  }

  if (state_ == MissionState::HighAlignDrop ||
    state_ == MissionState::HighAlignLand)
  {
    if (!vision_fresh || !has_height_) {
      state_start_time_ = now_time;
      publishMotionHold(true);
      return;
    }
    publishMotionHold(false);
    if ((now_time - state_start_time_).seconds() >= pre_descent_alignment_sec_) {
      Target descent_target = targets_[current_index_];
      if (state_ == MissionState::HighAlignLand) {
        descent_target.z_cm = landing_trigger_height_cm_;
        setState(MissionState::FollowLand);
      } else {
        descent_target.z_cm = drop_target_height_cm_;
        setState(MissionState::FollowDrop);
      }
      publishTarget(descent_target);
      publishVisualDescentState(true);
      RCLCPP_INFO(
        get_logger(),
        "High-altitude visual alignment completed; descent target is %.1f cm.",
        descent_target.z_cm);
    }
    return;
  }

  if (state_ == MissionState::FollowDrop) {
    if (!vision_fresh) {
      resetDropAlignmentCount();
      publishMotionHold(true);
      return;
    }
    if (!has_height_ || current_height_cm_ > drop_trigger_height_cm_) {
      resetDropAlignmentCount();
    }
    publishMotionHold(false);
    if (has_height_ &&
      current_height_cm_ <= drop_trigger_height_cm_ &&
      drop_aligned_frame_count_ >= drop_alignment_required_frames_)
    {
      task_x_cm_ = x_cm;
      task_y_cm_ = y_cm;
      task_yaw_deg_ = yaw_deg;
      search_segment_active_ = false;
      has_fine_data_ = false;
      resetDropAlignmentCount();
      publishMotionHold(true);
      publishVisualState(false);
      publishVisualDescentState(false);
      publishVisionFresh(false);
      last_vision_fresh_ = false;
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
  if (is_search && search_segment_active_ && vision_fresh && !returning_) {
    beginSearchTask(target);
    return;
  }

  publishMotionHold(false);
  if (!isFinalReturnLeg() && isCurrentTargetReached(x_cm, y_cm, yaw_deg)) {
    RCLCPP_INFO(
      get_logger(), "Reached waypoint %zu/%zu type=%d.",
      current_index_ + 1, targets_.size(), static_cast<int>(target.type));
    advanceTarget();
  }
}

void RouteTargetPublisherNode::beginSearchTask(const Target & target)
{
  targets_.resize(current_index_ + 1);
  search_segment_active_ = false;
  resetDropAlignmentCount();

  if (target.type == WaypointType::SearchLand) {
    setState(MissionState::HighAlignLand);
  } else {
    setState(MissionState::HighAlignDrop);
  }
  publishTarget(target);
  publishVisualState(true);
  publishVisualDescentState(false);
  publishMotionHold(false);
  RCLCPP_INFO(
    get_logger(),
    "Visual target detected on search leg %zu; holding %.1f cm for %.1f s alignment.",
    current_index_ + 1, target.z_cm, pre_descent_alignment_sec_);
}

void RouteTargetPublisherNode::updateSearchSegmentState()
{
  const bool should_search =
    !returning_ &&
    current_index_ < targets_.size() &&
    (targets_[current_index_].type == WaypointType::SearchDrop ||
    targets_[current_index_].type == WaypointType::SearchLand);

  if (should_search && !search_segment_active_) {
    search_segment_active_ = true;
    has_fine_data_ = false;
    resetDropAlignmentCount();
    last_fine_data_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_vision_fresh_ = false;
    publishVisionFresh(false);
    RCLCPP_INFO(
      get_logger(),
      "Entered continuous visual search segment at waypoint %zu; old vision data cleared.",
      current_index_ + 1);
  } else if (!should_search && search_segment_active_) {
    search_segment_active_ = false;
    has_fine_data_ = false;
    resetDropAlignmentCount();
    last_fine_data_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_vision_fresh_ = false;
    publishVisionFresh(false);
  }
  publishDroneStateIfReady();
}

void RouteTargetPublisherNode::startReturnRoute(bool takeoff_from_car)
{
  targets_.clear();
  if (takeoff_from_car) {
    const double climb_height_cm = post_task_return_waypoints_.front().z_cm;
    targets_.push_back(Target{
      task_x_cm_, task_y_cm_, climb_height_cm, task_yaw_deg_,
      WaypointType::Normal});
  }
  targets_.insert(
    targets_.end(),
    post_task_return_waypoints_.begin(),
    post_task_return_waypoints_.end());
  current_index_ = 0;
  returning_ = true;
  search_segment_active_ = false;
  has_fine_data_ = false;
  resetDropAlignmentCount();
  last_fine_data_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  last_vision_fresh_ = false;
  publishVisualState(false);
  publishVisualDescentState(false);
  publishVisionFresh(false);
  setState(MissionState::Returning);
  publishCurrentWaypointIndex();
  publishCurrentTarget();
  publishMotionHold(false);
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
  updateSearchSegmentState();
  publishCurrentTarget();
}

void RouteTargetPublisherNode::completeMission()
{
  search_segment_active_ = false;
  has_fine_data_ = false;
  resetDropAlignmentCount();
  last_vision_fresh_ = false;
  publishVisualState(false);
  publishVisualDescentState(false);
  publishMotionHold(true);
  publishVisionFresh(false);
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

bool RouteTargetPublisherNode::isFinalReturnLeg() const
{
  return state_ == MissionState::Returning &&
         returning_ &&
         !targets_.empty() &&
         current_index_ == targets_.size() - 1;
}

bool RouteTargetPublisherNode::isFineDataAligned() const
{
  return std::fabs(static_cast<double>(fine_error_x_px_)) <=
         drop_alignment_tolerance_px_ &&
         std::fabs(static_cast<double>(fine_error_y_px_)) <=
         drop_alignment_tolerance_px_;
}

void RouteTargetPublisherNode::resetDropAlignmentCount()
{
  if (drop_aligned_frame_count_ > 0) {
    RCLCPP_INFO(
      get_logger(), "Drop alignment count reset from %ld.",
      drop_aligned_frame_count_);
  }
  drop_aligned_frame_count_ = 0;
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

void RouteTargetPublisherNode::publishVisualDescentState(bool active)
{
  visual_descent_active_ = active;
  std_msgs::msg::Bool msg;
  msg.data = active;
  visual_descent_pub_->publish(msg);
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
  publishDroneStateIfReady();
}

uint8_t RouteTargetPublisherNode::desiredDroneState() const
{
  switch (state_) {
    case MissionState::WaitingRoute:
      return 0;
    case MissionState::Navigating:
      return search_segment_active_ ? 2 : 1;
    case MissionState::TakeoffHover:
      return 1;
    case MissionState::HighAlignDrop:
    case MissionState::HighAlignLand:
      return 2;
    case MissionState::FollowDrop:
      return has_height_ && current_height_cm_ < drone_state_action_height_cm_ ? 3 : 2;
    case MissionState::FollowLand:
      return has_height_ && current_height_cm_ < drone_state_action_height_cm_ ? 4 : 2;
    case MissionState::DropCommandPending:
      return 3;
    case MissionState::LandingStopPending:
    case MissionState::LandedHold:
    case MissionState::TakeoffCommandPending:
      return 4;
    case MissionState::WaitingTakeoffPose:
    case MissionState::Returning:
    case MissionState::FinalLandingStopPending:
    case MissionState::Completed:
      return 5;
    case MissionState::Error:
      return has_published_drone_state_ ? last_drone_state_ : 0;
  }
  return 0;
}

void RouteTargetPublisherNode::publishDroneStateIfReady(bool force)
{
  if (!drone_state_enabled_) {
    return;
  }
  const uint8_t drone_state = desiredDroneState();
  if (drone_state < 1 || drone_state > 5) {
    return;
  }
  if (!force && has_published_drone_state_ && drone_state == last_drone_state_) {
    return;
  }

  std_msgs::msg::UInt8 msg;
  msg.data = drone_state;
  drone_state_pub_->publish(msg);
  last_drone_state_ = drone_state;
  has_published_drone_state_ = true;
  RCLCPP_INFO(
    get_logger(), "Published /drone_state=%u.",
    static_cast<unsigned>(drone_state));
}

const char * RouteTargetPublisherNode::stateName(MissionState state)
{
  switch (state) {
    case MissionState::WaitingRoute: return "WAITING_ROUTE";
    case MissionState::Navigating: return "NAVIGATING";
    case MissionState::TakeoffHover: return "TAKEOFF_HOVER";
    case MissionState::HighAlignDrop: return "HIGH_ALIGN_DROP";
    case MissionState::HighAlignLand: return "HIGH_ALIGN_LAND";
    case MissionState::FollowDrop: return "FOLLOW_DROP";
    case MissionState::FollowLand: return "FOLLOW_LAND";
    case MissionState::DropCommandPending: return "DROP_COMMAND_PENDING";
    case MissionState::LandingStopPending: return "LANDING_STOP_PENDING";
    case MissionState::LandedHold: return "LANDED_HOLD";
    case MissionState::TakeoffCommandPending: return "TAKEOFF_COMMAND_PENDING";
    case MissionState::WaitingTakeoffPose: return "WAITING_TAKEOFF_POSE";
    case MissionState::Returning: return "RETURNING";
    case MissionState::FinalLandingStopPending: return "FINAL_LANDING_STOP_PENDING";
    case MissionState::Completed: return "COMPLETED";
    case MissionState::Error: return "ERROR";
  }
  return "UNKNOWN";
}

double RouteTargetPublisherNode::normalizeAngleDeg(double angle_deg)
{
  return angles::to_degrees(angles::normalize_angle(angles::from_degrees(angle_deg)));
}

Target RouteTargetPublisherNode::parseWaypoint(
  const std::string & text, WaypointType waypoint_type)
{
  if (text.size() < 2 || text.front() != '(' || text.back() != ')') {
    throw std::runtime_error(
            "waypoint must use '(x_cm y_cm z_cm yaw_deg)' format: " + text);
  }

  std::istringstream stream(text.substr(1, text.size() - 2));
  Target target;
  target.type = waypoint_type;
  if (!(stream >> target.x_cm >> target.y_cm >> target.z_cm >> target.yaw_deg)) {
    throw std::runtime_error(
            "waypoint must contain exactly four numbers: " + text);
  }
  std::string trailing;
  if (stream >> trailing) {
    throw std::runtime_error(
            "waypoint contains extra data after four numbers: " + text);
  }
  if (!std::isfinite(target.x_cm) ||
    !std::isfinite(target.y_cm) ||
    !std::isfinite(target.z_cm) ||
    !std::isfinite(target.yaw_deg))
  {
    throw std::runtime_error("waypoint values must be finite: " + text);
  }
  return target;
}

}  // namespace activity_control_pkg
