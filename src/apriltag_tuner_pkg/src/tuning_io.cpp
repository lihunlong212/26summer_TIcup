#include "apriltag_tuner_pkg/tuning_io.hpp"

#include <cstdlib>
#include <filesystem>
#include <stdexcept>

#include <opencv2/core.hpp>

namespace apriltag_tuner_pkg
{

namespace
{
void validate(const TuningValues & values)
{
  if (values.target_mode < 0 || values.target_mode > 1 ||
    values.target_id < 0 || values.target_id > 586)
  {
    throw std::invalid_argument("target mode/id is outside the 36h11 range");
  }
  if (values.adaptive_thresh_win_size_min < 3 ||
    values.adaptive_thresh_win_size_max < values.adaptive_thresh_win_size_min ||
    values.adaptive_thresh_win_size_step <= 0)
  {
    throw std::invalid_argument("invalid adaptive threshold windows");
  }
  if (values.min_marker_perimeter_rate <= 0.0 ||
    values.max_marker_perimeter_rate <= values.min_marker_perimeter_rate ||
    values.polygonal_approx_accuracy_rate <= 0.0 ||
    values.min_corner_distance_rate < 0.0)
  {
    throw std::invalid_argument("invalid marker geometry values");
  }
  if (values.corner_refinement_method < 0 ||
    values.corner_refinement_method > 3 ||
    values.corner_refinement_win_size <= 0 ||
    values.error_correction_rate < 0.0 ||
    values.error_correction_rate > 1.0)
  {
    throw std::invalid_argument("invalid corner refinement values");
  }
  if (values.perspective_remove_pixel_per_cell <= 0 ||
    values.perspective_remove_ignored_margin_per_cell < 0.0 ||
    values.perspective_remove_ignored_margin_per_cell >= 0.5 ||
    values.clahe_clip_limit < 0.0 ||
    values.sharpen_amount < 0.0 ||
    values.blur_radius < 0)
  {
    throw std::invalid_argument("invalid preprocessing values");
  }
}
}  // namespace

std::string defaultTuningFilePath()
{
  const char * home = std::getenv("HOME");
  if (home == nullptr || std::string(home).empty()) {
    return "/tmp/apriltag_detector.yaml";
  }
  return (
    std::filesystem::path(home) / ".config" / "nezha" /
    "apriltag_detector.yaml").string();
}

bool saveTuningFile(
  const std::string & path,
  const TuningValues & values,
  std::string * error_message)
{
  try {
    validate(values);
    const std::filesystem::path output_path(path);
    if (output_path.has_parent_path()) {
      std::filesystem::create_directories(output_path.parent_path());
    }
    cv::FileStorage output(path, cv::FileStorage::WRITE);
    if (!output.isOpened()) {
      throw std::runtime_error("cannot open output file");
    }
    output << "apriltag_tuner" << "{";
    output << "dictionary" << "DICT_APRILTAG_36h11";
    output << "target_mode" << values.target_mode;
    output << "target_id" << values.target_id;
    output << "adaptive_thresh_win_size_min"
           << values.adaptive_thresh_win_size_min;
    output << "adaptive_thresh_win_size_max"
           << values.adaptive_thresh_win_size_max;
    output << "adaptive_thresh_win_size_step"
           << values.adaptive_thresh_win_size_step;
    output << "adaptive_thresh_constant"
           << values.adaptive_thresh_constant;
    output << "min_marker_perimeter_rate"
           << values.min_marker_perimeter_rate;
    output << "max_marker_perimeter_rate"
           << values.max_marker_perimeter_rate;
    output << "polygonal_approx_accuracy_rate"
           << values.polygonal_approx_accuracy_rate;
    output << "min_corner_distance_rate"
           << values.min_corner_distance_rate;
    output << "min_distance_to_border" << values.min_distance_to_border;
    output << "corner_refinement_method" << values.corner_refinement_method;
    output << "corner_refinement_win_size" << values.corner_refinement_win_size;
    output << "error_correction_rate" << values.error_correction_rate;
    output << "detect_inverted_marker"
           << static_cast<int>(values.detect_inverted_marker);
    output << "perspective_remove_pixel_per_cell"
           << values.perspective_remove_pixel_per_cell;
    output << "perspective_remove_ignored_margin_per_cell"
           << values.perspective_remove_ignored_margin_per_cell;
    output << "clahe_clip_limit" << values.clahe_clip_limit;
    output << "sharpen_amount" << values.sharpen_amount;
    output << "blur_radius" << values.blur_radius;
    output << "}";
    output.release();
    return true;
  } catch (const std::exception & error) {
    if (error_message != nullptr) {
      *error_message = error.what();
    }
    return false;
  }
}

}  // namespace apriltag_tuner_pkg
