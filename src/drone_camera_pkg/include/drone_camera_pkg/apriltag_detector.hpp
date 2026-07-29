#ifndef DRONE_CAMERA_PKG__APRILTAG_DETECTOR_HPP_
#define DRONE_CAMERA_PKG__APRILTAG_DETECTOR_HPP_

#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

namespace drone_camera_pkg
{

struct AprilTagTuningSettings
{
  std::string dictionary_name{"DICT_APRILTAG_36h11"};
  int target_id{-1};
  int adaptive_thresh_win_size_min{3};
  int adaptive_thresh_win_size_max{53};
  int adaptive_thresh_win_size_step{4};
  double adaptive_thresh_constant{7.0};
  double min_marker_perimeter_rate{0.02};
  double max_marker_perimeter_rate{4.0};
  double polygonal_approx_accuracy_rate{0.03};
  double min_corner_distance_rate{0.03};
  int min_distance_to_border{3};
  int corner_refinement_method{3};
  int corner_refinement_win_size{5};
  double error_correction_rate{0.6};
  bool detect_inverted_marker{false};
  int perspective_remove_pixel_per_cell{6};
  double perspective_remove_ignored_margin_per_cell{0.13};
  double clahe_clip_limit{2.0};
  double sharpen_amount{1.0};
  int blur_radius{0};
};

class AprilTagDetector
{
public:
  AprilTagDetector(const std::string & dictionary_name, int target_id);

  bool loadTuningFile(const std::string & path, std::string * error_message = nullptr);
  const AprilTagTuningSettings & settings() const;
  bool tuningLoaded() const;

  bool detect(
    const cv::Mat & frame,
    cv::Point2f & selected_center,
    std::vector<int> * detected_ids = nullptr,
    std::vector<std::vector<cv::Point2f>> * detected_corners = nullptr) const;

private:
  static cv::aruco::PREDEFINED_DICTIONARY_NAME dictionaryFromName(
    const std::string & name);
  static void validateSettings(const AprilTagTuningSettings & settings);
  void applySettings(const AprilTagTuningSettings & settings);
  cv::Mat preprocess(const cv::Mat & frame) const;

  int target_id_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
  AprilTagTuningSettings settings_;
  bool tuning_loaded_{false};
};

}  // namespace drone_camera_pkg

#endif  // DRONE_CAMERA_PKG__APRILTAG_DETECTOR_HPP_
