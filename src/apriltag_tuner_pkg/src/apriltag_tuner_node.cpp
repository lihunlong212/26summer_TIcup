#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <fcntl.h>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <rclcpp/rclcpp.hpp>

#include "apriltag_tuner_pkg/tuning_io.hpp"

namespace apriltag_tuner_pkg
{

namespace
{
constexpr int kAprilTag36h11MaxId = 586;

int oddAtLeastThree(int value)
{
  value = std::max(value, 3);
  return value % 2 == 0 ? value + 1 : value;
}

std::string joinIds(const std::vector<int> & ids)
{
  if (ids.empty()) {
    return "none";
  }
  std::ostringstream stream;
  for (std::size_t index = 0; index < ids.size(); ++index) {
    if (index > 0) {
      stream << ',';
    }
    stream << ids[index];
  }
  return stream.str();
}

class TerminalKeyReader
{
public:
  TerminalKeyReader()
  {
    if (::isatty(STDIN_FILENO) == 0 ||
      ::tcgetattr(STDIN_FILENO, &original_termios_) != 0)
    {
      return;
    }
    original_flags_ = ::fcntl(STDIN_FILENO, F_GETFL, 0);
    termios raw = original_termios_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      return;
    }
    if (original_flags_ >= 0) {
      (void)::fcntl(STDIN_FILENO, F_SETFL, original_flags_ | O_NONBLOCK);
    }
    active_ = true;
  }

  ~TerminalKeyReader()
  {
    if (!active_) {
      return;
    }
    (void)::tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
    if (original_flags_ >= 0) {
      (void)::fcntl(STDIN_FILENO, F_SETFL, original_flags_);
    }
  }

  bool active() const
  {
    return active_;
  }

  int readKey() const
  {
    if (!active_) {
      return -1;
    }
    unsigned char key = 0;
    return ::read(STDIN_FILENO, &key, 1) == 1 ? static_cast<int>(key) : -1;
  }

private:
  termios original_termios_{};
  int original_flags_{-1};
  bool active_{false};
};
}  // namespace

class AprilTagTunerNode : public rclcpp::Node
{
public:
  AprilTagTunerNode()
  : Node("apriltag_tuner"),
    camera_device_(declare_parameter<std::string>("camera_device", "/dev/video0")),
    frame_width_(declare_parameter<int>("frame_width", 640)),
    frame_height_(declare_parameter<int>("frame_height", 480)),
    requested_fps_(declare_parameter<double>("fps", 15.0)),
    initial_target_id_(declare_parameter<int>("target_id", -1)),
    output_file_(defaultTuningFilePath()),
    dictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11)),
    detector_parameters_(cv::aruco::DetectorParameters::create())
  {
    if (initial_target_id_ < -1 || initial_target_id_ > kAprilTag36h11MaxId) {
      throw std::invalid_argument("target_id must be -1 or in [0, 586]");
    }
    if (!camera_.open(camera_device_)) {
      throw std::runtime_error("Failed to open camera device " + camera_device_);
    }
    if (frame_width_ > 0) {
      camera_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
    }
    if (frame_height_ > 0) {
      camera_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
    }
    if (requested_fps_ > 0.0) {
      camera_.set(cv::CAP_PROP_FPS, requested_fps_);
    }

    resetControls();
    target_mode_ = initial_target_id_ < 0 ? 0 : 1;
    target_id_ = std::max(initial_target_id_, 0);
    createWindowsAndTrackbars();

    RCLCPP_INFO(
      get_logger(),
      "Independent AprilTag tuner: device=%s actual=%.0fx%.0f@%.1f output=%s",
      camera_device_.c_str(),
      camera_.get(cv::CAP_PROP_FRAME_WIDTH),
      camera_.get(cv::CAP_PROP_FRAME_HEIGHT),
      camera_.get(cv::CAP_PROP_FPS),
      output_file_.c_str());
    RCLCPP_INFO(
      get_logger(), "Keys without Enter: s=save r=reset space=pause q/Esc=quit");
    if (!terminal_keys_.active()) {
      RCLCPP_WARN(
        get_logger(),
        "stdin is not a TTY; use keys while the preview window has focus, "
        "or run with 'ros2 run apriltag_tuner_pkg apriltag_tuner_node'.");
    }

    last_fps_time_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&AprilTagTunerNode::tick, this));
  }

  ~AprilTagTunerNode() override
  {
    if (camera_.isOpened()) {
      camera_.release();
    }
    cv::destroyAllWindows();
  }

private:
  void resetControls()
  {
    adaptive_min_ = 3;
    adaptive_max_ = 53;
    adaptive_step_ = 4;
    adaptive_constant_x10_ = 70;
    min_perimeter_x1000_ = 20;
    max_perimeter_x100_ = 400;
    polygon_accuracy_x1000_ = 30;
    min_corner_distance_x1000_ = 30;
    min_distance_border_ = 3;
    corner_refinement_method_ = 3;
    corner_refinement_win_ = 5;
    error_correction_x100_ = 60;
    detect_inverted_ = 0;
    perspective_pixels_per_cell_ = 6;
    ignored_margin_x100_ = 13;
    clahe_x10_ = 20;
    sharpen_x10_ = 10;
    blur_radius_ = 0;
    show_rejected_ = 1;
    show_binary_ = 0;
  }

  void createWindowsAndTrackbars()
  {
    cv::namedWindow(preview_window_, cv::WINDOW_NORMAL);
    cv::namedWindow(controls_window_, cv::WINDOW_NORMAL);
    cv::resizeWindow(preview_window_, 960, 720);
    cv::resizeWindow(controls_window_, 620, 850);

    cv::createTrackbar("target mode 0:any 1:id", controls_window_, &target_mode_, 1);
    cv::createTrackbar("target id", controls_window_, &target_id_, kAprilTag36h11MaxId);
    cv::createTrackbar("adaptive min", controls_window_, &adaptive_min_, 101);
    cv::createTrackbar("adaptive max", controls_window_, &adaptive_max_, 151);
    cv::createTrackbar("adaptive step", controls_window_, &adaptive_step_, 30);
    cv::createTrackbar(
      "adaptive constant x10", controls_window_, &adaptive_constant_x10_, 200);
    cv::createTrackbar(
      "min perimeter x1000", controls_window_, &min_perimeter_x1000_, 200);
    cv::createTrackbar(
      "max perimeter x100", controls_window_, &max_perimeter_x100_, 800);
    cv::createTrackbar(
      "polygon accuracy x1000", controls_window_, &polygon_accuracy_x1000_, 150);
    cv::createTrackbar(
      "min corner dist x1000", controls_window_, &min_corner_distance_x1000_, 200);
    cv::createTrackbar("min border px", controls_window_, &min_distance_border_, 30);
    cv::createTrackbar(
      "corner refine 0-3", controls_window_, &corner_refinement_method_, 3);
    cv::createTrackbar(
      "corner refine win", controls_window_, &corner_refinement_win_, 30);
    cv::createTrackbar(
      "error correction x100", controls_window_, &error_correction_x100_, 100);
    cv::createTrackbar("detect inverted", controls_window_, &detect_inverted_, 1);
    cv::createTrackbar(
      "pixels per cell", controls_window_, &perspective_pixels_per_cell_, 20);
    cv::createTrackbar(
      "ignored margin x100", controls_window_, &ignored_margin_x100_, 49);
    cv::createTrackbar("CLAHE x10 (0=off)", controls_window_, &clahe_x10_, 100);
    cv::createTrackbar("sharpen x10", controls_window_, &sharpen_x10_, 40);
    cv::createTrackbar("blur radius", controls_window_, &blur_radius_, 5);
    cv::createTrackbar("show rejected", controls_window_, &show_rejected_, 1);
    cv::createTrackbar("show binary", controls_window_, &show_binary_, 1);
    syncTrackbars();
  }

  void syncTrackbars()
  {
    cv::setTrackbarPos("target mode 0:any 1:id", controls_window_, target_mode_);
    cv::setTrackbarPos("target id", controls_window_, target_id_);
    cv::setTrackbarPos("adaptive min", controls_window_, adaptive_min_);
    cv::setTrackbarPos("adaptive max", controls_window_, adaptive_max_);
    cv::setTrackbarPos("adaptive step", controls_window_, adaptive_step_);
    cv::setTrackbarPos(
      "adaptive constant x10", controls_window_, adaptive_constant_x10_);
    cv::setTrackbarPos(
      "min perimeter x1000", controls_window_, min_perimeter_x1000_);
    cv::setTrackbarPos(
      "max perimeter x100", controls_window_, max_perimeter_x100_);
    cv::setTrackbarPos(
      "polygon accuracy x1000", controls_window_, polygon_accuracy_x1000_);
    cv::setTrackbarPos(
      "min corner dist x1000", controls_window_, min_corner_distance_x1000_);
    cv::setTrackbarPos("min border px", controls_window_, min_distance_border_);
    cv::setTrackbarPos(
      "corner refine 0-3", controls_window_, corner_refinement_method_);
    cv::setTrackbarPos(
      "corner refine win", controls_window_, corner_refinement_win_);
    cv::setTrackbarPos(
      "error correction x100", controls_window_, error_correction_x100_);
    cv::setTrackbarPos("detect inverted", controls_window_, detect_inverted_);
    cv::setTrackbarPos(
      "pixels per cell", controls_window_, perspective_pixels_per_cell_);
    cv::setTrackbarPos(
      "ignored margin x100", controls_window_, ignored_margin_x100_);
    cv::setTrackbarPos("CLAHE x10 (0=off)", controls_window_, clahe_x10_);
    cv::setTrackbarPos("sharpen x10", controls_window_, sharpen_x10_);
    cv::setTrackbarPos("blur radius", controls_window_, blur_radius_);
    cv::setTrackbarPos("show rejected", controls_window_, show_rejected_);
    cv::setTrackbarPos("show binary", controls_window_, show_binary_);
  }

  TuningValues currentValues() const
  {
    TuningValues values;
    values.target_mode = std::clamp(target_mode_, 0, 1);
    values.target_id = std::clamp(target_id_, 0, kAprilTag36h11MaxId);
    values.adaptive_thresh_win_size_min = oddAtLeastThree(adaptive_min_);
    values.adaptive_thresh_win_size_max = std::max(
      oddAtLeastThree(adaptive_max_), values.adaptive_thresh_win_size_min);
    values.adaptive_thresh_win_size_step = std::max(adaptive_step_, 1);
    values.adaptive_thresh_constant =
      static_cast<double>(adaptive_constant_x10_) / 10.0;
    values.min_marker_perimeter_rate =
      std::max(static_cast<double>(min_perimeter_x1000_) / 1000.0, 0.001);
    values.max_marker_perimeter_rate = std::max(
      static_cast<double>(max_perimeter_x100_) / 100.0,
      values.min_marker_perimeter_rate + 0.01);
    values.polygonal_approx_accuracy_rate =
      std::max(static_cast<double>(polygon_accuracy_x1000_) / 1000.0, 0.001);
    values.min_corner_distance_rate =
      std::max(static_cast<double>(min_corner_distance_x1000_) / 1000.0, 0.0);
    values.min_distance_to_border = std::max(min_distance_border_, 0);
    values.corner_refinement_method =
      std::clamp(corner_refinement_method_, 0, 3);
    values.corner_refinement_win_size = std::max(corner_refinement_win_, 1);
    values.error_correction_rate =
      static_cast<double>(error_correction_x100_) / 100.0;
    values.detect_inverted_marker = detect_inverted_ != 0;
    values.perspective_remove_pixel_per_cell =
      std::max(perspective_pixels_per_cell_, 1);
    values.perspective_remove_ignored_margin_per_cell =
      static_cast<double>(ignored_margin_x100_) / 100.0;
    values.clahe_clip_limit = static_cast<double>(clahe_x10_) / 10.0;
    values.sharpen_amount = static_cast<double>(sharpen_x10_) / 10.0;
    values.blur_radius = std::max(blur_radius_, 0);
    return values;
  }

  void updateDetectorParameters()
  {
    const TuningValues values = currentValues();
    detector_parameters_->adaptiveThreshWinSizeMin =
      values.adaptive_thresh_win_size_min;
    detector_parameters_->adaptiveThreshWinSizeMax =
      values.adaptive_thresh_win_size_max;
    detector_parameters_->adaptiveThreshWinSizeStep =
      values.adaptive_thresh_win_size_step;
    detector_parameters_->adaptiveThreshConstant =
      values.adaptive_thresh_constant;
    detector_parameters_->minMarkerPerimeterRate =
      values.min_marker_perimeter_rate;
    detector_parameters_->maxMarkerPerimeterRate =
      values.max_marker_perimeter_rate;
    detector_parameters_->polygonalApproxAccuracyRate =
      values.polygonal_approx_accuracy_rate;
    detector_parameters_->minCornerDistanceRate =
      values.min_corner_distance_rate;
    detector_parameters_->minDistanceToBorder = values.min_distance_to_border;
    detector_parameters_->cornerRefinementMethod =
      values.corner_refinement_method;
    detector_parameters_->cornerRefinementWinSize =
      values.corner_refinement_win_size;
    detector_parameters_->errorCorrectionRate = values.error_correction_rate;
    detector_parameters_->detectInvertedMarker =
      values.detect_inverted_marker;
    detector_parameters_->perspectiveRemovePixelPerCell =
      values.perspective_remove_pixel_per_cell;
    detector_parameters_->perspectiveRemoveIgnoredMarginPerCell =
      values.perspective_remove_ignored_margin_per_cell;
  }

  cv::Mat preprocess(const cv::Mat & frame) const
  {
    const TuningValues values = currentValues();
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    if (values.clahe_clip_limit > 0.0) {
      cv::Mat equalized;
      auto clahe = cv::createCLAHE(values.clahe_clip_limit, cv::Size(8, 8));
      clahe->apply(gray, equalized);
      gray = equalized;
    }
    if (values.blur_radius > 0) {
      const int kernel = values.blur_radius * 2 + 1;
      cv::GaussianBlur(gray, gray, cv::Size(kernel, kernel), 0.0);
    }
    if (values.sharpen_amount > 0.0) {
      cv::Mat blurred;
      cv::Mat sharpened;
      cv::GaussianBlur(gray, blurred, cv::Size(0, 0), 1.0);
      cv::addWeighted(
        gray, 1.0 + values.sharpen_amount,
        blurred, -values.sharpen_amount, 0.0, sharpened);
      gray = sharpened;
    }
    return gray;
  }

  int selectDetection(
    const std::vector<int> & ids,
    const std::vector<std::vector<cv::Point2f>> & corners,
    const cv::Size & size,
    cv::Point2f & selected_center) const
  {
    const cv::Point2f image_center(
      static_cast<float>(size.width) * 0.5F,
      static_cast<float>(size.height) * 0.5F);
    int selected = -1;
    double closest = std::numeric_limits<double>::max();
    for (std::size_t index = 0; index < ids.size(); ++index) {
      if (target_mode_ == 1 && ids[index] != target_id_) {
        continue;
      }
      if (corners[index].size() != 4) {
        continue;
      }
      cv::Point2f center(0.0F, 0.0F);
      for (const auto & corner : corners[index]) {
        center += corner;
      }
      center *= 0.25F;
      const double distance = cv::norm(center - image_center);
      if (distance < closest) {
        closest = distance;
        selected = static_cast<int>(index);
        selected_center = center;
      }
    }
    return selected;
  }

  void drawStatus(
    cv::Mat & display,
    const std::vector<int> & ids,
    int selected,
    const cv::Point2f & selected_center,
    std::size_t rejected_count)
  {
    cv::aruco::drawDetectedMarkers(
      display, detected_corners_, ids, cv::Scalar(60, 230, 60));
    if (selected >= 0) {
      cv::circle(display, selected_center, 6, cv::Scalar(40, 40, 255), cv::FILLED);
      const int forward_error =
        static_cast<int>(std::lround(display.rows * 0.5 - selected_center.y));
      const int lateral_error =
        static_cast<int>(std::lround(display.cols * 0.5 - selected_center.x));
      std::ostringstream selected_text;
      selected_text << "SELECTED id=" << ids[static_cast<std::size_t>(selected)]
                    << " fine_data=(" << forward_error << ','
                    << lateral_error << ")px";
      cv::putText(
        display, selected_text.str(), cv::Point(12, 58),
        cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(40, 40, 255), 2, cv::LINE_AA);
    }

    std::ostringstream first_line;
    first_line << std::fixed << std::setprecision(1)
               << "FPS " << measured_fps_ << "  IDs " << joinIds(ids)
               << "  rejected " << rejected_count;
    cv::putText(
      display, first_line.str(), cv::Point(12, 28),
      cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    const TuningValues values = currentValues();
    std::ostringstream params_line;
    params_line << "thr=" << values.adaptive_thresh_win_size_min << '-'
                << values.adaptive_thresh_win_size_max << '/'
                << values.adaptive_thresh_win_size_step
                << " refine=" << values.corner_refinement_method
                << " CLAHE=" << values.clahe_clip_limit
                << " sharp=" << values.sharpen_amount;
    cv::putText(
      display, params_line.str(), cv::Point(12, display.rows - 18),
      cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
  }

  void showBinaryPreview(const cv::Mat & processed)
  {
    if (show_binary_ == 0) {
      if (binary_window_open_) {
        cv::destroyWindow(binary_window_);
        binary_window_open_ = false;
      }
      return;
    }
    if (!binary_window_open_) {
      cv::namedWindow(binary_window_, cv::WINDOW_NORMAL);
      binary_window_open_ = true;
    }
    cv::Mat binary;
    cv::adaptiveThreshold(
      processed, binary, 255.0, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
      cv::THRESH_BINARY_INV,
      detector_parameters_->adaptiveThreshWinSizeMax,
      detector_parameters_->adaptiveThreshConstant);
    cv::imshow(binary_window_, binary);
  }

  void updateMeasuredFps()
  {
    ++fps_frame_count_;
    const auto current_time = std::chrono::steady_clock::now();
    const double elapsed =
      std::chrono::duration<double>(current_time - last_fps_time_).count();
    if (elapsed >= 1.0) {
      measured_fps_ = static_cast<double>(fps_frame_count_) / elapsed;
      fps_frame_count_ = 0;
      last_fps_time_ = current_time;
    }
  }

  void saveSettings()
  {
    updateDetectorParameters();
    std::string error;
    if (!saveTuningFile(output_file_, currentValues(), &error)) {
      RCLCPP_ERROR(
        get_logger(), "Cannot save AprilTag tuning to %s: %s",
        output_file_.c_str(), error.c_str());
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Saved AprilTag tuning to %s", output_file_.c_str());
  }

  void handleKey(int key)
  {
    if (key < 0) {
      return;
    }
    key &= 0xFF;
    if (key == 'q' || key == 27) {
      RCLCPP_INFO(get_logger(), "Quit requested.");
      rclcpp::shutdown();
    } else if (key == 's') {
      saveSettings();
    } else if (key == 'r') {
      resetControls();
      target_mode_ = initial_target_id_ < 0 ? 0 : 1;
      target_id_ = std::max(initial_target_id_, 0);
      syncTrackbars();
      RCLCPP_INFO(get_logger(), "Restored tuner defaults.");
    } else if (key == ' ') {
      paused_ = !paused_;
      RCLCPP_INFO(
        get_logger(), "Camera image %s.", paused_ ? "paused" : "resumed");
    }
  }

  void tick()
  {
    updateDetectorParameters();
    if (!paused_ || frozen_frame_.empty()) {
      cv::Mat frame;
      if (!camera_.read(frame) || frame.empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Failed to read frame from %s", camera_device_.c_str());
        handleKey(cv::waitKey(1));
        handleKey(terminal_keys_.readKey());
        return;
      }
      frozen_frame_ = frame;
      updateMeasuredFps();
    }

    const cv::Mat processed = preprocess(frozen_frame_);
    detected_ids_.clear();
    detected_corners_.clear();
    rejected_corners_.clear();
    cv::aruco::detectMarkers(
      processed, dictionary_, detected_corners_, detected_ids_,
      detector_parameters_, rejected_corners_);

    cv::Mat display = frozen_frame_.clone();
    if (show_rejected_ != 0) {
      for (const auto & candidate : rejected_corners_) {
        if (candidate.size() != 4) {
          continue;
        }
        std::vector<cv::Point> polygon;
        for (const auto & point : candidate) {
          polygon.emplace_back(
            static_cast<int>(std::lround(point.x)),
            static_cast<int>(std::lround(point.y)));
        }
        cv::polylines(
          display, polygon, true, cv::Scalar(0, 220, 255), 1, cv::LINE_AA);
      }
    }

    cv::Point2f selected_center;
    const int selected = selectDetection(
      detected_ids_, detected_corners_, display.size(), selected_center);
    drawStatus(
      display, detected_ids_, selected, selected_center,
      rejected_corners_.size());
    showBinaryPreview(processed);
    cv::imshow(preview_window_, display);
    handleKey(cv::waitKey(1));
    for (int key = terminal_keys_.readKey(); key >= 0;
      key = terminal_keys_.readKey())
    {
      handleKey(key);
    }
  }

  std::string camera_device_;
  int frame_width_;
  int frame_height_;
  double requested_fps_;
  int initial_target_id_;
  std::string output_file_;

  TerminalKeyReader terminal_keys_;
  cv::VideoCapture camera_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;
  rclcpp::TimerBase::SharedPtr timer_;

  const std::string preview_window_{"AprilTag tuner preview"};
  const std::string controls_window_{"AprilTag tuner controls"};
  const std::string binary_window_{"AprilTag threshold preview"};
  bool binary_window_open_{false};
  bool paused_{false};
  cv::Mat frozen_frame_;

  int target_mode_{0};
  int target_id_{0};
  int adaptive_min_{3};
  int adaptive_max_{53};
  int adaptive_step_{4};
  int adaptive_constant_x10_{70};
  int min_perimeter_x1000_{20};
  int max_perimeter_x100_{400};
  int polygon_accuracy_x1000_{30};
  int min_corner_distance_x1000_{30};
  int min_distance_border_{3};
  int corner_refinement_method_{3};
  int corner_refinement_win_{5};
  int error_correction_x100_{60};
  int detect_inverted_{0};
  int perspective_pixels_per_cell_{6};
  int ignored_margin_x100_{13};
  int clahe_x10_{20};
  int sharpen_x10_{10};
  int blur_radius_{0};
  int show_rejected_{1};
  int show_binary_{0};

  std::vector<int> detected_ids_;
  std::vector<std::vector<cv::Point2f>> detected_corners_;
  std::vector<std::vector<cv::Point2f>> rejected_corners_;
  std::chrono::steady_clock::time_point last_fps_time_;
  int fps_frame_count_{0};
  double measured_fps_{0.0};
};

}  // namespace apriltag_tuner_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<apriltag_tuner_pkg::AprilTagTunerNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    std::fprintf(stderr, "apriltag_tuner_node failed: %s\n", error.what());
    exit_code = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
