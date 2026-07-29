#ifndef APRILTAG_TUNER_PKG__TUNING_IO_HPP_
#define APRILTAG_TUNER_PKG__TUNING_IO_HPP_

#include <string>

namespace apriltag_tuner_pkg
{

struct TuningValues
{
  int target_mode{0};
  int target_id{0};
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

std::string defaultTuningFilePath();
bool saveTuningFile(
  const std::string & path,
  const TuningValues & values,
  std::string * error_message = nullptr);

}  // namespace apriltag_tuner_pkg

#endif  // APRILTAG_TUNER_PKG__TUNING_IO_HPP_
