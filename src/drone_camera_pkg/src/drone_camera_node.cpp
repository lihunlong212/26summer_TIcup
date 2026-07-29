#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include "drone_camera_pkg/apriltag_detector.hpp"

namespace
{
std::string defaultTuningFile()
{
  const char * home = std::getenv("HOME");
  if (home == nullptr || std::string(home).empty()) {
    return "/tmp/apriltag_detector.yaml";
  }
  return (
    std::filesystem::path(home) / ".config" / "nezha" /
    "apriltag_detector.yaml").string();
}
}  // namespace

class DroneCameraNode : public rclcpp::Node
{
public:
  DroneCameraNode()
  : Node("drone_camera_node"),
    camera_device_(declare_parameter<std::string>("camera_device", "/dev/video0")),
    frame_width_(declare_parameter<int>("frame_width", 640)),
    frame_height_(declare_parameter<int>("frame_height", 480)),
    fps_(declare_parameter<double>("fps", 15.0)),
    show_preview_(declare_parameter<bool>("show_preview", false)),
    apriltag_dictionary_name_(
      declare_parameter<std::string>("apriltag_dictionary", "DICT_APRILTAG_36h11")),
    apriltag_target_id_(declare_parameter<int>("apriltag_target_id", -1)),
    tuning_file_(declare_parameter<std::string>("tuning_file", defaultTuningFile()))
  {
    apriltag_detector_ = std::make_unique<drone_camera_pkg::AprilTagDetector>(
      apriltag_dictionary_name_, apriltag_target_id_);
    std::string tuning_error;
    if (apriltag_detector_->loadTuningFile(tuning_file_, &tuning_error)) {
      RCLCPP_INFO(
        get_logger(),
        "Loaded AprilTag detector tuning: %s (dictionary=%s target_id=%d).",
        tuning_file_.c_str(),
        apriltag_detector_->settings().dictionary_name.c_str(),
        apriltag_detector_->settings().target_id);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "AprilTag tuning file '%s' was not applied: %s. Using built-in defaults.",
        tuning_file_.c_str(), tuning_error.c_str());
    }
    fine_data_pub_ =
      create_publisher<std_msgs::msg::Int32MultiArray>("/fine_data", rclcpp::QoS(10));

    if (!camera_.open(camera_device_)) {
      throw std::runtime_error("Failed to open camera device " + camera_device_);
    }
    if (frame_width_ > 0) {
      camera_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
    }
    if (frame_height_ > 0) {
      camera_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
    }
    if (fps_ > 0.0) {
      camera_.set(cv::CAP_PROP_FPS, fps_);
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(fps_, 1.0));
    frame_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&DroneCameraNode::frameTimerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "AprilTag camera started: device=%s size=%dx%d fps=%.1f dictionary=%s target_id=%d "
      "preview=%s, output=/fine_data",
      camera_device_.c_str(), frame_width_, frame_height_, fps_,
      apriltag_detector_->settings().dictionary_name.c_str(),
      apriltag_detector_->settings().target_id,
      show_preview_ ? "true" : "false");
  }

  ~DroneCameraNode() override
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (camera_.isOpened()) {
      camera_.release();
    }
    if (show_preview_) {
      cv::destroyAllWindows();
    }
  }

private:
  void publishFineData(const cv::Mat & frame, const cv::Point2f & center)
  {
    const float image_center_x = static_cast<float>(frame.cols) / 2.0F;
    const float image_center_y = static_cast<float>(frame.rows) / 2.0F;
    std_msgs::msg::Int32MultiArray msg;
    msg.data = {
      static_cast<int32_t>(std::lround(image_center_y - center.y)),
      static_cast<int32_t>(std::lround(image_center_x - center.x))};
    fine_data_pub_->publish(msg);
  }

  void detectAprilTagAndPublish(cv::Mat & frame)
  {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::Point2f best_center;
    if (!apriltag_detector_->detect(frame, best_center, &ids, &corners)) {
      return;
    }

    publishFineData(frame, best_center);
    if (show_preview_) {
      cv::aruco::drawDetectedMarkers(frame, corners, ids);
      cv::circle(frame, best_center, 4, cv::Scalar(0, 0, 255), cv::FILLED);
    }
  }

  void frameTimerCallback()
  {
    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(camera_mutex_);
      if (!camera_.isOpened() || !camera_.read(frame) || frame.empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000, "Failed to read camera frame.");
        return;
      }
    }

    detectAprilTagAndPublish(frame);
    if (show_preview_) {
      cv::imshow("apriltag_preview", frame);
      cv::waitKey(1);
    }
  }

  std::string camera_device_;
  int frame_width_;
  int frame_height_;
  double fps_;
  bool show_preview_;
  std::string apriltag_dictionary_name_;
  int apriltag_target_id_;
  std::string tuning_file_;

  std::mutex camera_mutex_;
  cv::VideoCapture camera_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr fine_data_pub_;
  rclcpp::TimerBase::SharedPtr frame_timer_;
  std::unique_ptr<drone_camera_pkg::AprilTagDetector> apriltag_detector_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DroneCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
