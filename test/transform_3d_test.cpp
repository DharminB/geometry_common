#include "gtest/gtest.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "geometry_common/TransformMatrix3D.h"

using kelo::geometry_common::TransformMatrix3D;

TEST(TransformMatrix3D, multiplication)
{
    TransformMatrix3D tf1(1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f);
    TransformMatrix3D tf2; // identity transformation
    TransformMatrix3D tf3 = tf1 * tf2;
    EXPECT_NEAR(tf3.x(), 1.0f, 1e-3f);
    EXPECT_NEAR(tf3.y(), 2.0f, 1e-3f);
    EXPECT_NEAR(tf3.z(), 3.0f, 1e-3f);
    EXPECT_NEAR(tf3.roll(), 0.1f, 1e-3f);
    EXPECT_NEAR(tf3.pitch(), 0.2f, 1e-3f);
    EXPECT_NEAR(tf3.yaw(), 0.3f, 1e-3f);
}

TEST(TransformMatrix3D, transformStamped)
{
    geometry_msgs::msg::TransformStamped ts;
    ts.transform.translation.x = 1.0f;
    ts.transform.translation.y = 2.0f;
    ts.transform.translation.z = 3.0f;
    TransformMatrix3D tf(ts);
    EXPECT_NEAR(tf.x(), 1.0f, 1e-3f);
    EXPECT_NEAR(tf.y(), 2.0f, 1e-3f);
    EXPECT_NEAR(tf.z(), 3.0f, 1e-3f);
    EXPECT_NEAR(tf.roll(), 0.0f, 1e-3f);
    EXPECT_NEAR(tf.pitch(), 0.0f, 1e-3f);
    EXPECT_NEAR(tf.yaw(), 0.0f, 1e-3f);
}
