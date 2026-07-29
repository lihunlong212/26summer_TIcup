#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include <opencv2/core.hpp>

#include "apriltag_tuner_pkg/tuning_io.hpp"

TEST(AprilTagTuningIoTest, SavesProductionCompatibleYaml)
{
  const auto output =
    std::filesystem::temp_directory_path() / "codex_apriltag_tuning_test.yaml";
  apriltag_tuner_pkg::TuningValues values;
  values.target_mode = 1;
  values.target_id = 7;
  values.adaptive_thresh_constant = 9.5;
  values.clahe_clip_limit = 3.0;

  std::string error;
  ASSERT_TRUE(
    apriltag_tuner_pkg::saveTuningFile(output.string(), values, &error))
    << error;

  cv::FileStorage input(output.string(), cv::FileStorage::READ);
  ASSERT_TRUE(input.isOpened());
  const cv::FileNode root = input["apriltag_tuner"];
  ASSERT_FALSE(root.empty());
  EXPECT_EQ(static_cast<int>(root["target_mode"]), 1);
  EXPECT_EQ(static_cast<int>(root["target_id"]), 7);
  EXPECT_NEAR(
    static_cast<double>(root["adaptive_thresh_constant"]), 9.5, 1e-6);
  EXPECT_NEAR(static_cast<double>(root["clahe_clip_limit"]), 3.0, 1e-6);
  input.release();
  std::filesystem::remove(output);
}
