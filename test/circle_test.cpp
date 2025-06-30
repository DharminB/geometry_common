#include "gtest/gtest.h"

#include "geometry_common/Circle.h"

using kelo::geometry_common::Circle;
using kelo::geometry_common::Point2D;

TEST(CircleTest, constructor)
{
    Circle circle;
    EXPECT_NEAR(circle.x, 0.0f, 1e-3f);
    EXPECT_NEAR(circle.y, 0.0f, 1e-3f);
    EXPECT_NEAR(circle.r, 0.0f, 1e-3f);

    Circle circle2(1.0f, 2.0f, 1.5f);
    EXPECT_NEAR(circle2.x, 1.0f, 1e-3f);
    EXPECT_NEAR(circle2.y, 2.0f, 1e-3f);
    EXPECT_NEAR(circle2.r, 1.5f, 1e-3f);
}

TEST(CircleTest, fromPoints)
{
    Point2D p1(0.0f, 0.0f);
    Point2D p2(2.0f, 0.0f);
    Point2D p3(1.0f, 1.0f);
    Circle circle;

    EXPECT_TRUE(Circle::fromPoints(p1, p2, p3, circle));
    EXPECT_EQ(circle, Circle(1.0f, 0.0f, 1.0f));

    Point2D p4(1.0f, 0.0f);

    EXPECT_FALSE(Circle::fromPoints(p1, p2, p4, circle));
}
