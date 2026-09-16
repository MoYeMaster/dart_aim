#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include <vector>

#include "detector/image_utils.hpp"

namespace rm_auto_aim
{
namespace
{
sensor_msgs::msg::Image makeBgrImage()
{
  sensor_msgs::msg::Image image;
  image.height = 1;
  image.width = 2;
  image.encoding = "bgr8";
  image.is_bigendian = 0;
  image.step = 6;
  image.data = {
    0, 0, 255,
    0, 255, 0};
  return image;
}
}  // namespace

TEST(ImageUtilsTest, ConvertsBgr8ToMono8ForColorCamera)
{
  const cv::Mat gray = makeDetectorInputImage(makeBgrImage(), 1);

  ASSERT_EQ(gray.type(), CV_8UC1);
  ASSERT_EQ(gray.rows, 1);
  ASSERT_EQ(gray.cols, 2);
  EXPECT_NEAR(gray.at<unsigned char>(0, 0), 76, 1);
  EXPECT_NEAR(gray.at<unsigned char>(0, 1), 150, 1);
}

TEST(ImageUtilsTest, KeepsMono8ForGrayCamera)
{
  sensor_msgs::msg::Image image;
  image.height = 1;
  image.width = 2;
  image.encoding = "mono8";
  image.is_bigendian = 0;
  image.step = 2;
  image.data = {11, 23};

  const cv::Mat gray = makeDetectorInputImage(image, 0);

  ASSERT_EQ(gray.type(), CV_8UC1);
  EXPECT_EQ(gray.at<unsigned char>(0, 0), 11);
  EXPECT_EQ(gray.at<unsigned char>(0, 1), 23);
}
}  // namespace rm_auto_aim
