#include "gtest/gtest.h"

#include "geometry_msgs/msg/quaternion.hpp"

#include "geometry_common/Pose2D.h"

TEST(Pose2DTest, constructor)
{
    kelo::geometry_common::Pose2D pose1;
    EXPECT_FLOAT_EQ(pose1.x, 0.0f) << "pose1.x is not zero";
    EXPECT_FLOAT_EQ(pose1.y, 0.0f) << "pose1.y is not zero";
    EXPECT_FLOAT_EQ(pose1.theta, 0.0f) << "pose1.theta is not zero";

    kelo::geometry_common::Pose2D pose2(1.0f, 2.0f, 1.5f);
    EXPECT_FLOAT_EQ(pose2.x, 1.0f) << "pose2.x is not 1.0f";
    EXPECT_FLOAT_EQ(pose2.y, 2.0f) << "pose2.y is not 2.0f";
    EXPECT_FLOAT_EQ(pose2.theta, 1.5f) << "pose2.theta is not 1.5f";
}
