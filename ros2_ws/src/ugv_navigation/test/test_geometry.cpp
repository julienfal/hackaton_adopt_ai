#include <gtest/gtest.h>
#include "ugv_navigation/geometry.hpp"
TEST(ugv_navigation, transformToRobotFrame_test)
{
    Point2D result = transformToRobotFrame({2, 1, 0}, {5, 3});
    EXPECT_NEAR(3, result.x, 1e-5);
    EXPECT_NEAR(2, result.y, 1e-5);
}
