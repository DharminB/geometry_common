#include <vector>

#include "gtest/gtest.h"

#include "geometry_common/Polygon2D.h"

using kelo::geometry_common::Point2D;
using kelo::geometry_common::Polygon2D;

TEST(Polygon2DTest, isConvex)
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

TEST(Polygon2DTest, isApproximatelyConvex)
{
    Polygon2D convex_polygon(
    {
        Point2D(0.0f, 0.0f),
        Point2D(5.0f, 0.0f),
        Point2D(4.0f, 4.0f),
        Point2D(0.0f, 3.0f)
    });
    EXPECT_EQ(convex_polygon.isConvex(), true);

    Polygon2D approximately_convex_polygon(
    {
        Point2D(0.0f, 0.0f),
        Point2D(3.0f, 0.01f),   // slightly offset point
        Point2D(5.0f, 0.0f),
        Point2D(4.0f, 4.0f),
        Point2D(0.0f, 3.0f)
    });
    EXPECT_EQ(approximately_convex_polygon.isConvex(), false);
    EXPECT_EQ(approximately_convex_polygon.isApproximatelyConvex(), true);

    Polygon2D repeated_points_convex_polygon(
    {
        Point2D(0.0f, 0.0f),    // repeated point at the start
        Point2D(0.0f, 0.0f),
        Point2D(5.0f, 0.0f),
        Point2D(4.0f, 4.0f),
        Point2D(4.0f, 4.0f),    // repeated point in between
        Point2D(0.0f, 3.0f)
    });
    EXPECT_EQ(repeated_points_convex_polygon.isConvex(), false);
    EXPECT_EQ(repeated_points_convex_polygon.isApproximatelyConvex(), true);

    Polygon2D concave_polygon(
    {
        Point2D(0.0f, 0.0f),
        Point2D(3.0f, 0.05f),
        Point2D(5.0f, 0.0f),
        Point2D(4.0f, 4.0f),
        Point2D(0.0f, 3.0f)
    });
    EXPECT_EQ(concave_polygon.isConvex(), false);
    EXPECT_EQ(concave_polygon.isApproximatelyConvex(), false);
}

TEST(Polygon2DTest, calcConvexHullOfPolygons)
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

TEST(Polygon2DTest, calcInflatedPolygon)
{
    Polygon2D polygon(
    {
        Point2D(0.0f, 0.0f),
        Point2D(5.0f, 0.0f),
        Point2D(5.0f, 5.0f),
        Point2D(0.0f, 5.0f)
    });
    Polygon2D inflated_polygon = polygon.calcInflatedPolygon(0.1f);

    EXPECT_EQ(inflated_polygon.size(), 4u);
    EXPECT_EQ(inflated_polygon[0], Point2D(-0.1f, -0.1f));
    EXPECT_EQ(inflated_polygon[1], Point2D( 5.1f, -0.1f));
    EXPECT_EQ(inflated_polygon[2], Point2D( 5.1f,  5.1f));
    EXPECT_EQ(inflated_polygon[3], Point2D(-0.1f,  5.1f));

    Polygon2D polygon2(
    {
        Point2D(0.0f, 0.0f),
        Point2D(5.0f, 0.0f),
        Point2D(4.0f, 4.0f),
        Point2D(0.0f, 3.0f)
    });
    Polygon2D inflated_polygon2 = polygon2.calcInflatedPolygon(0.1f);

    EXPECT_EQ(inflated_polygon2.size(), 4u);
    EXPECT_EQ(inflated_polygon2[0], Point2D(-0.1f, -0.1f));
    EXPECT_EQ(inflated_polygon2[1], Point2D( 5.128f, -0.1f));
    EXPECT_EQ(inflated_polygon2[2], Point2D( 4.073f,  4.121f));
    EXPECT_EQ(inflated_polygon2[3], Point2D(-0.1f,  3.078f));
}
