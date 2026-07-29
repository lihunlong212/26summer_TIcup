#include "drone_camera_pkg/apriltag_detector.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace drone_camera_pkg
{

namespace
{
constexpr int kAprilTag36h11MaxId = 586;

template<typename ValueT>
void readRequired(
  const cv::FileNode & root, const char * key, ValueT & value)
{
  const cv::FileNode node = root[key];
  if (node.empty()) {
    throw std::runtime_error(std::string("missing key: ") + key);
  }
  node >> value;
}
}  // namespace

AprilTagDetector::AprilTagDetector(
  const std::string & dictionary_name, int target_id)
: target_id_(target_id),
  dictionary_(
    cv::aruco::getPredefinedDictionary(dictionaryFromName(dictionary_name))),
  parameters_(cv::aruco::DetectorParameters::create())
{
  settings_.dictionary_name = dictionary_name;
  settings_.target_id = target_id;
  applySettings(settings_);
}

bool AprilTagDetector::loadTuningFile(
  const std::string & path, std::string * error_message)
{
  try {
    if (path.empty()) {
      throw std::runtime_error("tuning file path is empty");
    }
    if (!std::filesystem::exists(path)) {
      throw std::runtime_error("file does not exist");
    }

    cv::FileStorage input(path, cv::FileStorage::READ);
    if (!input.isOpened()) {
      throw std::runtime_error("cannot open file");
    }
    const cv::FileNode root = input["apriltag_tuner"];
    if (root.empty() || !root.isMap()) {
      throw std::runtime_error("missing apriltag_tuner map");
    }

    AprilTagTuningSettings loaded = settings_;
    int target_mode = 0;
    int target_id = 0;
    int detect_inverted = 0;
    readRequired(root, "dictionary", loaded.dictionary_name);
    readRequired(root, "target_mode", target_mode);
    readRequired(root, "target_id", target_id);
    readRequired(
      root, "adaptive_thresh_win_size_min",
      loaded.adaptive_thresh_win_size_min);
    readRequired(
      root, "adaptive_thresh_win_size_max",
      loaded.adaptive_thresh_win_size_max);
    readRequired(
      root, "adaptive_thresh_win_size_step",
      loaded.adaptive_thresh_win_size_step);
    readRequired(
      root, "adaptive_thresh_constant",
      loaded.adaptive_thresh_constant);
    readRequired(
      root, "min_marker_perimeter_rate",
      loaded.min_marker_perimeter_rate);
    readRequired(
      root, "max_marker_perimeter_rate",
      loaded.max_marker_perimeter_rate);
    readRequired(
      root, "polygonal_approx_accuracy_rate",
      loaded.polygonal_approx_accuracy_rate);
    readRequired(
      root, "min_corner_distance_rate",
      loaded.min_corner_distance_rate);
    readRequired(
      root, "min_distance_to_border",
      loaded.min_distance_to_border);
    readRequired(
      root, "corner_refinement_method",
      loaded.corner_refinement_method);
    readRequired(
      root, "corner_refinement_win_size",
      loaded.corner_refinement_win_size);
    readRequired(root, "error_correction_rate", loaded.error_correction_rate);
    readRequired(root, "detect_inverted_marker", detect_inverted);
    readRequired(
      root, "perspective_remove_pixel_per_cell",
      loaded.perspective_remove_pixel_per_cell);
    readRequired(
      root, "perspective_remove_ignored_margin_per_cell",
      loaded.perspective_remove_ignored_margin_per_cell);
    readRequired(root, "clahe_clip_limit", loaded.clahe_clip_limit);
    readRequired(root, "sharpen_amount", loaded.sharpen_amount);
    readRequired(root, "blur_radius", loaded.blur_radius);

    if (target_mode != 0 && target_mode != 1) {
      throw std::runtime_error("target_mode must be 0 (any) or 1 (specific ID)");
    }
    loaded.target_id = target_mode == 0 ? -1 : target_id;
    loaded.detect_inverted_marker = detect_inverted != 0;
    validateSettings(loaded);
    applySettings(loaded);
    tuning_loaded_ = true;
    return true;
  } catch (const std::exception & error) {
    if (error_message != nullptr) {
      *error_message = error.what();
    }
    tuning_loaded_ = false;
    return false;
  }
}

const AprilTagTuningSettings & AprilTagDetector::settings() const
{
  return settings_;
}

bool AprilTagDetector::tuningLoaded() const
{
  return tuning_loaded_;
}

void AprilTagDetector::validateSettings(
  const AprilTagTuningSettings & settings)
{
  (void)dictionaryFromName(settings.dictionary_name);
  if (settings.target_id < -1 || settings.target_id > kAprilTag36h11MaxId) {
    throw std::invalid_argument("target_id must be -1 or in [0, 586]");
  }
  if (settings.adaptive_thresh_win_size_min < 3 ||
    settings.adaptive_thresh_win_size_max <
    settings.adaptive_thresh_win_size_min ||
    settings.adaptive_thresh_win_size_step <= 0)
  {
    throw std::invalid_argument("invalid adaptive threshold window settings");
  }
  if (settings.min_marker_perimeter_rate <= 0.0 ||
    settings.max_marker_perimeter_rate <= settings.min_marker_perimeter_rate ||
    settings.polygonal_approx_accuracy_rate <= 0.0 ||
    settings.min_corner_distance_rate < 0.0)
  {
    throw std::invalid_argument("invalid marker geometry settings");
  }
  if (settings.min_distance_to_border < 0 ||
    settings.corner_refinement_method < 0 ||
    settings.corner_refinement_method > 3 ||
    settings.corner_refinement_win_size <= 0 ||
    settings.error_correction_rate < 0.0 ||
    settings.error_correction_rate > 1.0)
  {
    throw std::invalid_argument("invalid corner/error-correction settings");
  }
  if (settings.perspective_remove_pixel_per_cell <= 0 ||
    settings.perspective_remove_ignored_margin_per_cell < 0.0 ||
    settings.perspective_remove_ignored_margin_per_cell >= 0.5 ||
    settings.clahe_clip_limit < 0.0 ||
    settings.sharpen_amount < 0.0 ||
    settings.blur_radius < 0)
  {
    throw std::invalid_argument("invalid perspective/preprocessing settings");
  }
}

void AprilTagDetector::applySettings(
  const AprilTagTuningSettings & settings)
{
  validateSettings(settings);
  settings_ = settings;
  target_id_ = settings.target_id;
  dictionary_ = cv::aruco::getPredefinedDictionary(
    dictionaryFromName(settings.dictionary_name));
  parameters_ = cv::aruco::DetectorParameters::create();
  parameters_->adaptiveThreshWinSizeMin =
    settings.adaptive_thresh_win_size_min;
  parameters_->adaptiveThreshWinSizeMax =
    settings.adaptive_thresh_win_size_max;
  parameters_->adaptiveThreshWinSizeStep =
    settings.adaptive_thresh_win_size_step;
  parameters_->adaptiveThreshConstant = settings.adaptive_thresh_constant;
  parameters_->minMarkerPerimeterRate = settings.min_marker_perimeter_rate;
  parameters_->maxMarkerPerimeterRate = settings.max_marker_perimeter_rate;
  parameters_->polygonalApproxAccuracyRate =
    settings.polygonal_approx_accuracy_rate;
  parameters_->minCornerDistanceRate = settings.min_corner_distance_rate;
  parameters_->minDistanceToBorder = settings.min_distance_to_border;
  parameters_->cornerRefinementMethod = settings.corner_refinement_method;
  parameters_->cornerRefinementWinSize = settings.corner_refinement_win_size;
  parameters_->errorCorrectionRate = settings.error_correction_rate;
  parameters_->detectInvertedMarker = settings.detect_inverted_marker;
  parameters_->perspectiveRemovePixelPerCell =
    settings.perspective_remove_pixel_per_cell;
  parameters_->perspectiveRemoveIgnoredMarginPerCell =
    settings.perspective_remove_ignored_margin_per_cell;
}

cv::Mat AprilTagDetector::preprocess(const cv::Mat & frame) const
{
  cv::Mat gray;
  if (frame.channels() == 1) {
    gray = frame.clone();
  } else if (frame.channels() == 3) {
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  } else if (frame.channels() == 4) {
    cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
  } else {
    throw std::invalid_argument("AprilTag frame must have 1, 3 or 4 channels");
  }

  if (settings_.clahe_clip_limit > 0.0) {
    cv::Mat equalized;
    auto clahe = cv::createCLAHE(settings_.clahe_clip_limit, cv::Size(8, 8));
    clahe->apply(gray, equalized);
    gray = equalized;
  }
  if (settings_.blur_radius > 0) {
    const int kernel = settings_.blur_radius * 2 + 1;
    cv::GaussianBlur(gray, gray, cv::Size(kernel, kernel), 0.0);
  }
  if (settings_.sharpen_amount > 0.0) {
    cv::Mat blurred;
    cv::Mat sharpened;
    cv::GaussianBlur(gray, blurred, cv::Size(0, 0), 1.0);
    cv::addWeighted(
      gray, 1.0 + settings_.sharpen_amount,
      blurred, -settings_.sharpen_amount, 0.0, sharpened);
    gray = sharpened;
  }
  return gray;
}

bool AprilTagDetector::detect(
  const cv::Mat & frame,
  cv::Point2f & selected_center,
  std::vector<int> * detected_ids,
  std::vector<std::vector<cv::Point2f>> * detected_corners) const
{
  if (frame.empty()) {
    return false;
  }
  const cv::Mat processed = preprocess(frame);
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(processed, dictionary_, corners, ids, parameters_);

  const cv::Point2f image_center(
    static_cast<float>(frame.cols) / 2.0F,
    static_cast<float>(frame.rows) / 2.0F);
  int best_index = -1;
  double best_distance = std::numeric_limits<double>::max();
  for (std::size_t index = 0; index < ids.size(); ++index) {
    if (target_id_ >= 0 && ids[index] != target_id_) {
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
    const double distance =
      std::hypot(center.x - image_center.x, center.y - image_center.y);
    if (distance < best_distance) {
      best_distance = distance;
      best_index = static_cast<int>(index);
      selected_center = center;
    }
  }

  if (detected_ids != nullptr) {
    *detected_ids = std::move(ids);
  }
  if (detected_corners != nullptr) {
    *detected_corners = std::move(corners);
  }
  return best_index >= 0;
}

cv::aruco::PREDEFINED_DICTIONARY_NAME AprilTagDetector::dictionaryFromName(
  const std::string & name)
{
  if (name == "DICT_APRILTAG_16h5") {
    return cv::aruco::DICT_APRILTAG_16h5;
  }
  if (name == "DICT_APRILTAG_25h9") {
    return cv::aruco::DICT_APRILTAG_25h9;
  }
  if (name == "DICT_APRILTAG_36h10") {
    return cv::aruco::DICT_APRILTAG_36h10;
  }
  if (name == "DICT_APRILTAG_36h11") {
    return cv::aruco::DICT_APRILTAG_36h11;
  }
  throw std::invalid_argument("Unsupported AprilTag dictionary: " + name);
}

}  // namespace drone_camera_pkg
