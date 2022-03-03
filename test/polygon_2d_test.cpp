#include <gtest/gtest.h>

#include <vector>

#include <geometry_common/Polygon2D.h>

using kelo::geometry_common::Point2D;
using kelo::geometry_common::Polygon2D;

TEST(UtilsTest, isPolygonConvex)
{
    Polygon2D convex_polygon(
    {
        Point2D(0.0f, 0.0f),
        Point2D(5.0f, 0.0f),
        Point2D(4.0f, 4.0f),
        Point2D(0.0f, 3.0f)
    });
    EXPECT_EQ(convex_polygon.isConvex(), true)
        << "Convex polygon is not found to be convex.";

    Polygon2D concave_polygon(
    {
        Point2D(0.0f, 0.0f),
        Point2D(5.0f, 0.0f),
        Point2D(1.0f, 1.0f),
        Point2D(0.0f, 5.0f)
    });
    EXPECT_EQ(concave_polygon.isConvex(), false)
        << "Concave polygon is not found to be concave.";

    Polygon2D star_polygon(
    {
        Point2D(0.0f, 0.0f),
        Point2D(2.0f, 5.0f),
        Point2D(4.0f, 0.0f),
        Point2D(0.0f, 3.0f),
        Point2D(4.0f, 3.0f),
    });
    EXPECT_EQ(star_polygon.isConvex(), false)
        << "Star polygon is not found to be concave.";
}

TEST(UtilsTest, calcConvexHullOfPolygons)
{
    Polygon2D polygon_a(
    {
        Point2D(0.0f, 0.0f),
        Point2D(5.0f, 0.0f),
        Point2D(5.0f, 4.0f),
        Point2D(0.0f, 4.0f)
    });
    Polygon2D polygon_b(
    {
        Point2D(3.0f, 2.0f),
        Point2D(9.0f, 1.0f),
        Point2D(9.0f, 3.0f)
    });
    Polygon2D convex_hull = Polygon2D::calcConvexHullOfPolygons(polygon_a, polygon_b);

    EXPECT_EQ(convex_hull.size(), 6u);
    EXPECT_EQ(convex_hull[0], polygon_a[0]);
    EXPECT_EQ(convex_hull[1], polygon_a[1]);
    EXPECT_EQ(convex_hull[2], polygon_b[1]);
    EXPECT_EQ(convex_hull[3], polygon_b[2]);
    EXPECT_EQ(convex_hull[4], polygon_a[2]);
    EXPECT_EQ(convex_hull[5], polygon_a[3]);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
