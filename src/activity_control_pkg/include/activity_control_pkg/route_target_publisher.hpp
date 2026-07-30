#ifndef ACTIVITY_CONTROL_PKG__ROUTE_TARGET_PUBLISHER_HPP_
#define ACTIVITY_CONTROL_PKG__ROUTE_TARGET_PUBLISHER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace activity_control_pkg
{

enum class WaypointType : int
{
  Normal = 1,
  SearchDrop = 2,
  SearchLand = 3,
};

struct Target
{
  double x_cm{0.0};
  double y_cm{0.0};
  double z_cm{0.0};
  double yaw_deg{0.0};
  WaypointType type{WaypointType::Normal};
};

class RouteTargetPublisherNode : public rclcpp::Node
{
public:
  explicit RouteTargetPublisherNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum class MissionState
  {
    WaitingRoute,
    Navigating,
    HighAlignDrop,
    HighAlignLand,
    FollowDrop,
    FollowLand,
    DropCommandPending,
    LandingStopPending,
    LandedHold,
    TakeoffCommandPending,
    WaitingTakeoffPose,
    Returning,
    Completed,
    Error,
  };

  void flyChoiceCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void fineDataCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void targetVelocityCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void serialCommandResultCallback(
    const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  void monitorTimerCallback();

  std::vector<Target> loadConfiguredRoute(uint8_t fly_choice) const;
  void declareRouteParameters(uint8_t fly_choice);
  void loadRoute(uint8_t fly_choice);
  void beginSearchTask(const Target & target);
  void updateSearchSegmentState();
  void startReturnRoute(bool takeoff_from_car);
  void advanceTarget();
  void completeMission();
  void resetDropAlignmentCount();

  bool getCurrentPose(double & x_cm, double & y_cm, double & yaw_deg);
  bool isCurrentTargetReached(double x_cm, double y_cm, double yaw_deg) const;
  bool hasFreshFineData(const rclcpp::Time & now_time) const;
  bool isFineDataAligned() const;

  void publishCurrentTarget();
  void publishTarget(const Target & target);
  void publishVisualState(bool active);
  void publishMotionHold(bool active);
  void publishVisualDescentState(bool active);
  void publishVisionFresh(bool fresh);
  void publishDroneStateIfReady(bool force = false);
  void publishCurrentWaypointIndex();
  void publishSerialByteCommand(uint8_t frame_id, uint8_t value);
  void setState(MissionState state);

  uint8_t desiredDroneState() const;
  static const char * stateName(MissionState state);
  static double normalizeAngleDeg(double angle_deg);
  static Target parseWaypoint(
    const std::string & text, WaypointType waypoint_type);

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visual_takeover_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_hold_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visual_descent_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_fresh_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr drone_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr waypoint_index_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr fly_choice_status_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_command_pub_;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr fly_choice_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr fine_data_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
    target_velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr
    serial_command_result_sub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string map_frame_;
  std::string robot_frame_;
  double position_tolerance_cm_;
  double height_tolerance_cm_;
  double yaw_tolerance_deg_;
  double fine_data_timeout_sec_;
  double pre_descent_alignment_sec_;
  double drop_alignment_height_cm_;
  double drop_alignment_tolerance_px_;
  int64_t drop_alignment_required_frames_;
  double landed_hold_sec_;
  double landing_trigger_height_cm_;
  double return_height_cm_;
  double drone_state_action_height_cm_;

  std::vector<Target> targets_;
  std::size_t current_index_;
  uint8_t fly_choice_;
  MissionState state_;
  bool returning_;
  bool search_segment_active_;
  bool has_height_;
  double current_height_cm_;
  bool has_fine_data_;
  int32_t fine_error_x_px_;
  int32_t fine_error_y_px_;
  int64_t drop_aligned_frame_count_;
  rclcpp::Time last_fine_data_time_;
  rclcpp::Time state_start_time_;
  double task_x_cm_;
  double task_y_cm_;
  double task_yaw_deg_;
  bool visual_active_;
  bool motion_hold_active_;
  bool visual_descent_active_;
  bool last_vision_fresh_;
  bool drone_state_enabled_;
  bool has_published_drone_state_;
  uint8_t last_drone_state_;
};

}  // namespace activity_control_pkg

#endif  // ACTIVITY_CONTROL_PKG__ROUTE_TARGET_PUBLISHER_HPP_
