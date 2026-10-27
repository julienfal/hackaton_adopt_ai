#include <gtest/gtest.h>
#include "ugv_navigation/geometry.hpp"
TEST(ugv_navigation, transformToRobotFrame_test)
{
    Point2D result = transformToRobotFrame({2, 1, 0}, {5, 3});
    EXPECT_NEAR(3, result.x, 1e-5);
    EXPECT_NEAR(2, result.y, 1e-5);

    Point2D result2 = transformToRobotFrame({0, 0, 0}, {3, 2});
    EXPECT_NEAR(3, result2.x, 1e-5);
    EXPECT_NEAR(2, result2.y, 1e-5);

    Point2D result3 = transformToRobotFrame({2, 1, M_PI/2}, {5, 3});
    EXPECT_NEAR(2, result3.x, 1e-5);
    EXPECT_NEAR(-3, result3.y, 1e-5);

    Point2D result4 = transformToRobotFrame({2, 1, -M_PI/2}, {5, 3});
    EXPECT_NEAR(-2, result4.x, 1e-5);
    EXPECT_NEAR(3, result4.y, 1e-5);

    Point2D result5 = transformToRobotFrame({2, 1, M_PI/2}, {2, 1});
    EXPECT_NEAR(0, result5.x, 1e-5);
    EXPECT_NEAR(0, result5.y, 1e-5);

}

TEST(ugv_navigation, closestPointReturnsNearestIndex)
{
    const std::vector<Point2D> path{{0, 0}, {2, 0}, {4, 0}};
    const auto index = findClosestPointIndex({2.2, 0}, path);

    ASSERT_TRUE(index.has_value());
    EXPECT_EQ(*index, 1u);
}

TEST(ugv_navigation, closestPointUsesBothCoordinates)
{
    const std::vector<Point2D> path{{0, 5}, {1, 0}};
    const auto index = findClosestPointIndex({0, 0}, path);

    ASSERT_TRUE(index.has_value());
    EXPECT_EQ(*index, 1u);
}

TEST(ugv_navigation, closestPointKeepsFirstIndexOnTie)
{
    const std::vector<Point2D> path{{0, 0}, {2, 0}};
    const auto index = findClosestPointIndex({1, 0}, path);

    ASSERT_TRUE(index.has_value());
    EXPECT_EQ(*index, 0u);
}

TEST(ugv_navigation, closestPointHandlesSinglePoint)
{
    const std::vector<Point2D> path{{4, 3}};
    const auto index = findClosestPointIndex({0, 0}, path);

    ASSERT_TRUE(index.has_value());
    EXPECT_EQ(*index, 0u);
}

TEST(ugv_navigation, closestPointHandlesEmptyPath)
{
    const auto index = findClosestPointIndex({0, 0}, {});

    EXPECT_FALSE(index.has_value());
}

TEST(ugv_navigation, lookaheadReturnsFirstPointBeyondThreshold)
{
    const std::vector<Point2D> path{{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}};
    const auto index = findLookaheadPointIndex({0, 0}, path, 2.5);

    ASSERT_TRUE(index.has_value());
    EXPECT_EQ(*index, 3u);
}

TEST(ugv_navigation, lookaheadIncludesPointExactlyAtThreshold)
{
    const std::vector<Point2D> path{{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}};
    const auto index = findLookaheadPointIndex({0, 0}, path, 2.0);

    ASSERT_TRUE(index.has_value());
    EXPECT_EQ(*index, 2u);
}

TEST(ugv_navigation, lookaheadReturnsNoResultWhenRemainingPathIsTooShort)
{
    const std::vector<Point2D> path{{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}};
    const auto index = findLookaheadPointIndex({0, 0}, path, 5.0);

    EXPECT_FALSE(index.has_value());
}

TEST(ugv_navigation, lookaheadRejectsZeroDistance)
{
    const std::vector<Point2D> path{{0, 0}, {2, 0}};
    const auto index = findLookaheadPointIndex({0, 0}, path, 0.0);

    EXPECT_FALSE(index.has_value());
}

TEST(ugv_navigation, lookaheadRejectsNegativeDistance)
{
    const std::vector<Point2D> path{{0, 0}, {2, 0}};
    const auto index = findLookaheadPointIndex({0, 0}, path, -1.0);

    EXPECT_FALSE(index.has_value());
}

TEST(ugv_navigation, lookaheadHandlesEmptyPath)
{
    const auto index = findLookaheadPointIndex({0, 0}, {}, 2.0);

    EXPECT_FALSE(index.has_value());
}

TEST(ugv_navigation, lookaheadSearchesForwardAndMeasuresFromRobot)
{
    const std::vector<Point2D> path{{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}};
    const auto index = findLookaheadPointIndex({2.2, 0}, path, 2.0);

    // Index 0 is far enough but precedes the closest point.
    // Index 4 is only 1.8 m from the robot; index 5 is 2.8 m away.
    ASSERT_TRUE(index.has_value());
    EXPECT_EQ(*index, 5u);
}

TEST(ugv_navigation, lookaheadUsesEuclideanDistanceInBothCoordinates)
{
    const std::vector<Point2D> path{{1, 1}, {2, 2}, {4, 5}, {7, 9}};
    const auto index = findLookaheadPointIndex({1, 1}, path, 5.0);

    // The displacement to index 2 is (3, 4), giving a distance of 5 m.
    ASSERT_TRUE(index.has_value());
    EXPECT_EQ(*index, 2u);
}

TEST(ugv_navigation, curvatureIsZeroForStraightTarget)
{
    const auto curvature = computeCurvature({2, 0});

    ASSERT_TRUE(curvature.has_value());
    EXPECT_NEAR(*curvature, 0.0, 1e-12);
}

TEST(ugv_navigation, curvatureIsPositiveForLeftTarget)
{
    const auto curvature = computeCurvature({2, 2});

    ASSERT_TRUE(curvature.has_value());
    EXPECT_NEAR(*curvature, 0.5, 1e-12);
}

TEST(ugv_navigation, curvatureIsNegativeForRightTarget)
{
    const auto curvature = computeCurvature({2, -2});

    ASSERT_TRUE(curvature.has_value());
    EXPECT_NEAR(*curvature, -0.5, 1e-12);
}

TEST(ugv_navigation, curvatureAcceptsZeroXWithNonzeroY)
{
    const auto curvature = computeCurvature({0, 2});

    ASSERT_TRUE(curvature.has_value());
    EXPECT_NEAR(*curvature, 1.0, 1e-12);
}

TEST(ugv_navigation, curvatureRejectsOrigin)
{
    EXPECT_FALSE(computeCurvature({0, 0}).has_value());
}

TEST(ugv_navigation, curvatureRejectsTargetBelowMinimumDistance)
{
    EXPECT_FALSE(computeCurvature({0.5e-6, 0.5e-6}).has_value());
}

TEST(ugv_navigation, curvatureRejectsTargetAtMinimumDistance)
{
    EXPECT_FALSE(computeCurvature({1e-6, 0}).has_value());
}

TEST(ugv_navigation, curvatureAcceptsTargetAboveMinimumDistance)
{
    // This catches comparing squared distance against an unsquared threshold.
    const auto curvature = computeCurvature({2e-6, 0});

    ASSERT_TRUE(curvature.has_value());
    EXPECT_NEAR(*curvature, 0.0, 1e-12);
}
