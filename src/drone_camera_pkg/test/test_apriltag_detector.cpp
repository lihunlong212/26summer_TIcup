#include <gtest/gtest.h>

#include <filesystem>
#include <stdexcept>
#include <string>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "drone_camera_pkg/apriltag_detector.hpp"

namespace
{

cv::Mat makeTagFrame(int tag_id)
{
  const auto dictionary =
    cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
  cv::Mat marker;
  cv::aruco::drawMarker(dictionary, tag_id, 200, marker, 1);
  cv::Mat frame(400, 400, CV_8UC1, cv::Scalar(255));
  marker.copyTo(frame(cv::Rect(100, 100, marker.cols, marker.rows)));
  return frame;
}

std::filesystem::path writeTuningFile()
{
  const auto path =
    std::filesystem::temp_directory_path() /
    "codex_drone_camera_apriltag_tuning.yaml";
  cv::FileStorage output(path.string(), cv::FileStorage::WRITE);
  output << "apriltag_tuner" << "{";
  output << "dictionary" << "DICT_APRILTAG_36h11";
  output << "target_mode" << 1;
  output << "target_id" << 7;
  output << "adaptive_thresh_win_size_min" << 3;
  output << "adaptive_thresh_win_size_max" << 53;
  output << "adaptive_thresh_win_size_step" << 4;
  output << "adaptive_thresh_constant" << 7.0;
  output << "min_marker_perimeter_rate" << 0.02;
  output << "max_marker_perimeter_rate" << 4.0;
  output << "polygonal_approx_accuracy_rate" << 0.03;
  output << "min_corner_distance_rate" << 0.03;
  output << "min_distance_to_border" << 3;
  output << "corner_refinement_method" << 3;
  output << "corner_refinement_win_size" << 5;
  output << "error_correction_rate" << 0.6;
  output << "detect_inverted_marker" << 0;
  output << "perspective_remove_pixel_per_cell" << 6;
  output << "perspective_remove_ignored_margin_per_cell" << 0.13;
  output << "clahe_clip_limit" << 3.0;
  output << "sharpen_amount" << 0.5;
  output << "blur_radius" << 1;
  output << "}";
  output.release();
  return path;
}

TEST(AprilTagDetectorTest, DetectsAny36h11TagAndReturnsItsCenter)
{
  drone_camera_pkg::AprilTagDetector detector("DICT_APRILTAG_36h11", -1);
  cv::Point2f center;
  EXPECT_TRUE(detector.detect(makeTagFrame(7), center));
  EXPECT_NEAR(center.x, 199.5, 1.0);
  EXPECT_NEAR(center.y, 199.5, 1.0);
}

TEST(AprilTagDetectorTest, EnforcesConfiguredTargetId)
{
  drone_camera_pkg::AprilTagDetector matching("DICT_APRILTAG_36h11", 7);
  drone_camera_pkg::AprilTagDetector different("DICT_APRILTAG_36h11", 8);
  cv::Point2f center;
  EXPECT_TRUE(matching.detect(makeTagFrame(7), center));
  EXPECT_FALSE(different.detect(makeTagFrame(7), center));
}

TEST(AprilTagDetectorTest, RejectsUnknownDictionary)
{
  EXPECT_THROW(
    drone_camera_pkg::AprilTagDetector("NOT_A_DICTIONARY", -1),
    std::invalid_argument);
}

TEST(AprilTagDetectorTest, LoadsSavedTuningAndAppliesTargetAndPreprocessing)
{
  const auto tuning_file = writeTuningFile();
  drone_camera_pkg::AprilTagDetector detector("DICT_APRILTAG_36h11", -1);
  std::string error;
  ASSERT_TRUE(detector.loadTuningFile(tuning_file.string(), &error)) << error;
  EXPECT_TRUE(detector.tuningLoaded());
  EXPECT_EQ(detector.settings().target_id, 7);
  EXPECT_DOUBLE_EQ(detector.settings().clahe_clip_limit, 3.0);
  EXPECT_DOUBLE_EQ(detector.settings().sharpen_amount, 0.5);
  EXPECT_EQ(detector.settings().blur_radius, 1);

  cv::Point2f center;
  EXPECT_TRUE(detector.detect(makeTagFrame(7), center));
  EXPECT_FALSE(detector.detect(makeTagFrame(8), center));
  std::filesystem::remove(tuning_file);
}

TEST(AprilTagDetectorTest, InvalidTuningFallsBackToBuiltInSettings)
{
  const auto invalid_file =
    std::filesystem::temp_directory_path() /
    "codex_invalid_apriltag_tuning.yaml";
  cv::FileStorage output(invalid_file.string(), cv::FileStorage::WRITE);
  output << "not_apriltag_tuner" << "invalid";
  output.release();

  drone_camera_pkg::AprilTagDetector detector("DICT_APRILTAG_36h11", -1);
  std::string error;
  EXPECT_FALSE(detector.loadTuningFile(invalid_file.string(), &error));
  EXPECT_FALSE(error.empty());
  EXPECT_EQ(detector.settings().target_id, -1);
  cv::Point2f center;
  EXPECT_TRUE(detector.detect(makeTagFrame(8), center));
  std::filesystem::remove(invalid_file);
}

}  // namespace
