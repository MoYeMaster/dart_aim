#include <gtest/gtest.h>

#include <opencv2/imgproc.hpp>

#include "detector/detector.hpp"

namespace rm_auto_aim
{
namespace
{
cv::Mat makeTestImage()
{
  cv::Mat image = cv::Mat::zeros(240, 320, CV_8UC1);
  cv::circle(image, cv::Point(160, 120), 8, cv::Scalar(255), cv::FILLED);
  cv::rectangle(image, cv::Rect(20, 20, 45, 8), cv::Scalar(255), cv::FILLED);
  return image;
}
}  // namespace

TEST(BaseLightDetectorTest, DetectsCircularGuideLightAndRejectsElongatedBlob)
{
  Detector::Params params;
  params.threshold = 180;
  params.min_area = 20.0;
  params.max_area = 200.0;
  params.min_diameter_px = 4.0;
  params.max_diameter_px = 30.0;
  params.min_circularity = 0.65;
  params.min_fill_ratio = 0.55;

  Detector rm_auto_aim(params);
  const auto detections = rm_auto_aim.detect(makeTestImage());

  ASSERT_EQ(detections.size(), 1U);
  EXPECT_NEAR(detections.front().center.x, 160.0F, 1.0F);
  EXPECT_NEAR(detections.front().center.y, 120.0F, 1.0F);
}

TEST(BaseLightDetectorTest, EmptyImageProducesNoDetection)
{
  Detector rm_auto_aim;

  const auto detections = rm_auto_aim.detect(cv::Mat::zeros(120, 160, CV_8UC1));

  EXPECT_TRUE(detections.empty());
}
}  // namespace rm_auto_aim
