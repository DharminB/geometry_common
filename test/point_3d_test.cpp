#include "gtest/gtest.h"

#include "geometry_common/Point3D.h"

TEST(PointTest, constructor)
{
    kelo::geometry_common::Point3D p;
    EXPECT_EQ(p.x, 0.0f) << "p.x is not zero";
    EXPECT_EQ(p.y, 0.0f) << "p.y is not zero";
    EXPECT_EQ(p.z, 0.0f) << "p.z is not zero";

    kelo::geometry_common::Point3D pt(1.0f, 2.0f, 3.0f);
    EXPECT_EQ(pt.x, 1.0f) << "pt.x is not 1.0f";
    EXPECT_EQ(pt.y, 2.0f) << "pt.y is not 2.0f";
    EXPECT_EQ(pt.z, 3.0f) << "pt.z is not 3.0f";
}
