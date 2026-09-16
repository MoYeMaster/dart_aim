#include <gtest/gtest.h>

#include "detector/dart_state.hpp"

namespace rm_auto_aim
{

TEST(DartStateTest, ClassifiesTargetToTheLeft)
{
  const auto result = classifyDartState(400.0, 1000.0, 0.03);

  EXPECT_EQ(result.states, -1);
  EXPECT_FALSE(result.is_middle);
}

TEST(DartStateTest, ClassifiesTargetToTheRight)
{
  const auto result = classifyDartState(600.0, 1000.0, 0.03);

  EXPECT_EQ(result.states, 1);
  EXPECT_FALSE(result.is_middle);
}

TEST(DartStateTest, ClassifiesTargetInsideMiddleTolerance)
{
  const auto result = classifyDartState(520.0, 1000.0, 0.03);

  EXPECT_EQ(result.states, 0);
  EXPECT_TRUE(result.is_middle);
}

TEST(DartStateTest, InvalidImageWidthDoesNotReportMiddle)
{
  const auto result = classifyDartState(500.0, 0.0, 0.03);

  EXPECT_EQ(result.states, 0);
  EXPECT_FALSE(result.is_middle);
}
}  // namespace rm_auto_aim
