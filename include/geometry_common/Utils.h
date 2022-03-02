#ifndef KELO_GEOMETRY_COMMON_UTILS_H
#define KELO_GEOMETRY_COMMON_UTILS_H

#include <vector>
#include <string>

#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <geometry_common/Pose2D.h>
#include <geometry_common/Point.h>
#include <geometry_common/LineSegment.h>

namespace kelo::geometry_common
{

class Utils
{
    public:

        static float roundFloat(
                float value,
                unsigned decimal_places);

        static Point getMeanPoint(
                const PointCloud& points,
                unsigned start_index,
                unsigned end_index);

        static Point getMeanPoint(
                const PointCloud& points);

        template <typename T>
        static Pose2D getMeanPose(
                const T& poses);

        static Point getClosestPoint(
                const PointCloud& points,
                float x=0.0f,
                float y=0.0f,
                float z=0.0f);

        static std::vector<std::vector<Point> > clusterPoints(
                const PointCloud& points,
                float cluster_distance_threshold=0.1f,
                size_t min_cluster_size=3);

        static std::vector<std::vector<Point> > clusterOrderedPoints(
                const PointCloud& points,
                float cluster_distance_threshold=0.1f,
                size_t min_cluster_size=3);

        /* 
         * Source: https://stackoverflow.com/a/2922778/10460994
         */
        static bool isPointInPolygon(
                const std::vector<Point>& polygon,
                const Point& point);

        static bool isFootprintSafe(
                const std::vector<Point>& footprint,
                const PointCloud& points);

        /*
         * 2D transform (returns a 2D point which only has x and y component)
         */
        static Point getTransformedPoint(
                const Pose2D& tf,
                const Point& pt);

        /*
         * 3D transform (returns a 3D point which has x, y and z component)
         */
        static Point getTransformedPoint(
                const std::vector<float>& tf_mat,
                const Point& pt);

        static void transformPoint2D(
                const std::vector<float>& tf_mat,
                Point& pt);

        static void transformPoint(
                const std::vector<float>& tf_mat,
                Point& pt);

        static Pose2D getTransformedPose(
                const std::vector<float> tf_mat,
                const Pose2D& pose);

        static Pose2D getTransformedPose(
                const Pose2D& tf,
                const Pose2D& pose);

        static std::vector<Point> getTransformedFootprint(
                const Pose2D& tf,
                const std::vector<Point>& footprint);

        static std::vector<Pose2D> getTrajectory(
                const Pose2D& vel,
                size_t num_of_poses,
                float future_time);

        static std::vector<float> multiplyMatrixToVector(
                const std::vector<float>& mat_a,
                const std::vector<float>& vec_b);

        static std::vector<float> multiplyMatrices(
                const std::vector<float>& mat_a,
                const std::vector<float>& mat_b,
                size_t N);

        static std::vector<float> get2DTransformMat(
                float x,
                float y,
                float theta);

        static std::vector<float> getTransformMat(
                float x,
                float y,
                float z,
                float roll,
                float pitch,
                float yaw);

        static float getShortestAngle(
                float angle1,
                float angle2);

        static void findPerpendicularLineAt(
                float m,
                float c,
                const Point& p,
                float& perpendicular_m,
                float& perpendicular_c);

        static float distToLineSquared(
                float m,
                float c,
                const Point& p);

        static Point getProjectedPointOnLine(
                float m,
                float c,
                const Point& p);

        static Point getProjectedPointOnSegment(
                const Point& a,
                const Point& b,
                const Point& p,
                bool is_segment);

        static Point getProjectedPointOnMajorAxis(
                float m,
                float c,
                const Point& p);

        /*
         * find the distance of point p from a line formed by points a and b
         * source: https://stackoverflow.com/a/1501725/10460994
         */
        static float distToLineSquared(
                const Point& a,
                const Point& b,
                const Point& p,
                bool is_segment = false);

        static float distToLineSegmentSquared(
                const LineSegment& line_segment,
                const Point& p);

        static float fitLineRANSAC(
                const PointCloud& pts,
                unsigned start_index,
                unsigned end_index,
                float& m,
                float& c,
                float delta = 0.2f,
                size_t itr_limit = 10);

        static float fitLineRANSAC(
                const PointCloud& pts,
                float& m, float& c,
                float delta = 0.2f,
                size_t itr_limit = 10);

        static float fitLineSegmentRANSAC(
                const PointCloud& pts,
                unsigned start_index,
                unsigned end_index,
                LineSegment& line_segment,
                float delta = 0.2f,
                size_t itr_limit = 10);

        static float fitLineSegmentRANSAC(
                const PointCloud& pts,
                LineSegment& line_segment,
                float delta = 0.2f,
                size_t itr_limit = 10);

        static std::vector<LineSegment> fitLineSegmentsRANSAC(
                const PointCloud& pts,
                float score_threshold = 0.9f,
                float delta = 0.2f,
                size_t itr_limit = 10);

        static float fitLineRegression(
                const PointCloud& pts,
                unsigned start_index,
                unsigned end_index,
                LineSegment& line_segment);

        static float fitLineRegression(
                const PointCloud& pts,
                LineSegment& line_segment);

        static std::vector<LineSegment> piecewiseRegression(
                const PointCloud& pts,
                float error_threshold = 0.1f);

        static std::vector<LineSegment> piecewiseRegressionSplit(
                const PointCloud& pts,
                float error_threshold = 0.1f);

        static void mergeCloseLines(
                std::vector<LineSegment>& line_segments,
                float distance_threshold = 0.2,
                float angle_threshold = 0.2);

        static void mergeCloseLinesBF(
                std::vector<LineSegment>& line_segments,
                float distance_threshold = 0.2,
                float angle_threshold = 0.2);

        static std::vector<LineSegment> fitLineSegments(
                const std::vector<Point> &pts,
                float regression_error_threshold = 0.1f,
                float distance_threshold = 0.2,
                float angle_threshold = 0.2);

        static float clip(
                float value,
                float max_limit,
                float min_limit);

        static float signedClip(
                float value,
                float max_limit,
                float min_limit);

        static float clipAngle(
                float raw_angle);

        static Pose2D applyVelLimits(
                const Pose2D& vel,
                const Pose2D& max_vel,
                const Pose2D& min_vel);

        static Pose2D applyAccLimits(
                const Pose2D& cmd_vel,
                const Pose2D& curr_vel,
                const Pose2D& max_acc,
                float loop_rate);

        static float linearInterpolate(
                float src,
                float target,
                float t);

        static sensor_msgs::PointCloud convertToROSPC(
                const PointCloud& pc,
                const std::string& frame);

        static PointCloud convertFromROSPC(
                const sensor_msgs::PointCloud& pc);

        /*
         * `row_sub_sample_factor` and `col_sub_sample_factor` parameterise how
         * many points are skipped.
         * e.g. if both their values is 1, no points are skipped
         *      if both their values are 2 and it is an organised cloud, the
         *      resulting cloud will be 1/4th the input cloud
         */
        static PointCloud convertFromROSPC(
                const sensor_msgs::PointCloud2& cloud_msg,
                size_t row_sub_sample_factor = 1,
                size_t col_sub_sample_factor = 1);

        static PointCloud convertFromROSScan(
                const sensor_msgs::LaserScan& scan);

        static std::vector<float> getInverted2DTransformMat(
                const Pose2D& tf);

        static std::vector<float> getInverted2DTransformMat(
                const std::vector<float>& tf);

        static Pose2D getInverted2DTransformPose(
                const Pose2D& tf);

        static float getPerpendicularAngle(
                float angle);

        static float getReverseAngle(
                float angle);

        /*
         * Since the angles wrap around, the rule of thumb here is that the sector
         * considered is the one where you move counter-clockwise from min_angle and
         * all angles are always represented from -pi to pi
         */
        static bool isAngleWithinBounds(
                float angle,
                float max_angle,
                float min_angle);

        static std::vector<Point> generatePerpendicularPoints(
                const Pose2D& start,
                const Pose2D& end,
                const Point& pt,
                float max_perp_dist = 3.0f,
                float step_size = 0.1f);

        static float getAngleBetweenPoints(
                const Point& a,
                const Point& b,
                const Point& c);

        static bool isPolygonConvex(
                const std::vector<Point>& polygon);

        /*
         * Note: area can be negative (the sign informs if the polygon is
         * clockwise or not)
         */
        static float calcPolygonArea(
                const std::vector<Point>& polygon);

        /*
         * Find convex hull from the union of 2 polygons.
         * source: https://en.wikipedia.org/wiki/Graham_scan#Pseudocode
         */
        static std::vector<Point> calcConvexHullOfPolygons(
                const std::vector<Point>& polygon_a,
                const std::vector<Point>& polygon_b);

        static nav_msgs::Path getPathMsgFromTrajectory(
                const std::vector<Pose2D>& trajectory,
                const std::string& frame);

        static visualization_msgs::Marker getPolygonAsMarker(
                const std::vector<Point>& polygon,
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float line_width = 0.05f);

        static visualization_msgs::Marker getGeometricPathAsMarker(
                const std::vector<Pose2D>& geometric_path,
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float line_width = 0.05f);

        static visualization_msgs::Marker getPointcloudAsMarker(
                const PointCloud& cloud,
                const std::string& frame,
                float diameter = 0.05f,
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f);

        static visualization_msgs::Marker getStringAsMarker(
                const std::string& string_label,
                const std::string& frame,
                float red = 0.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float size = 0.2f);

        static std::string getMatrixAsString(
                const std::vector<float>& mat,
                size_t N);
};

} // namespace kelo::geometry_common

#endif // KELO_GEOMETRY_COMMON_UTILS_H
