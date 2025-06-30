/******************************************************************************
 * Copyright (c) 2022
 * KELO Robotics GmbH
 *
 * Author:
 * Dharmin Bakaraniya
 * Sushant Chavan
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Locomotec nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/

#pragma once

#include <vector>
#include <string>

#include "geometry_msgs/msg/point32.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "geometry_common/Pose2D.h"
#include "geometry_common/Point3D.h"
#include "geometry_common/Circle.h"
#include "geometry_common/LineSegment2D.h"
#include "geometry_common/Enums.h"

namespace kelo
{
namespace geometry_common
{

/**
 * @brief A utility class containing `static` functions useful for dealing with
 * geometry related data.
 *
 */
class Utils
{
    public:
        /**
         * @brief round floating point number to specified decimal places
         *
         * @param value floating point number to be rounded
         * @param decimal_places decimal places to which the number needs to be
         * rounded
         * @return float rounded floating point value
         */
        static float roundFloat(
                float value,
                unsigned decimal_places);

        /**
         * @brief Calculate mean/average of vector of 2D/3D points between
         * specified indexes
         *
         * @tparam T template parameter Point2D or Point3D
         * @param points vector of points whose mean needs to be calculated
         * @param start_index start index in the vector of points where the mean
         * calculation should begin (including itself)
         * @param end_index end index in the vector of points where the mean
         * calculation should end (including itself)
         * @return T mean Point2D or Point3D
         */
        template <typename T>
        static T calcMeanPoint(
                const std::vector<T>& points,
                unsigned start_index,
                unsigned end_index);

        /**
         * @brief Calculate mean/average of vector of 2D/3D points
         *
         * @tparam T template parameter Point2D or Point3D
         * @param points vector of points whose mean needs to be calculated
         * @return T mean Point2D or Point3D
         */
        template <typename T>
        static T calcMeanPoint(
                const std::vector<T>& points);

        /**
         * @brief Calculate mean/average of collection of Pose2D
         *
         * @tparam T template parameter for collection (std::vector, std::deque, etc)
         * @param poses collection of poses whose mean needs to be calculated
         * @return Pose2D mean pose
         */
        template <typename T>
        static Pose2D calcMeanPose(
                const T& poses);

        /**
         * @brief Calculate the closest/nearest Point2D/Point3D object from
         * specified point
         *
         * @tparam T template parameter Point2D or Point3D
         * @param points collection of points from which closest point will be
         * chosen
         * calculated
         * @param pt specified point from which the closest point calculation are made
         * @return T closest point from the specified point in the collection
         */
        template <typename T>
        static T calcClosestPoint(
                const std::vector<T>& points,
                const T& pt = T());

        /**
         * @brief cluster 2D points based on distance
         *
         * @param points collection of points which is needed for clustering
         * @param cluster_distance_threshold minimum distance threshold between
         * any two clusters
         * @param min_cluster_size clusters containing points smaller than this
         * limit will be removed
         * @return std::vector<PointCloud2D> collection of clusters. Each
         * PointCloud2D is a cluster
         */
        static std::vector<PointCloud2D> clusterPoints(
                const PointCloud2D& points,
                float cluster_distance_threshold = 0.1f,
                size_t min_cluster_size = 3);

        /**
         * @brief cluster 2D points that are ordered based on distance.
         * More efficient than `Utils::clusterPoints`.
         *
         * @param points ordered collection of points which is needed for
         * clustering. Ordering refers to angle. Useful when points are
         * generated by a laser scanner.
         * @param cluster_distance_threshold minimum distance threshold between
         * any two clusters
         * @param min_cluster_size clusters containing points smaller than this
         * limit will be removed
         * @return std::vector<PointCloud2D> collection of clusters. Each
         * PointCloud2D is a cluster that is also itself ordered.
         */
        static std::vector<PointCloud2D> clusterOrderedPoints(
                const PointCloud2D& points,
                float cluster_distance_threshold = 0.1f,
                size_t min_cluster_size = 3);

        /**
         * @brief Sort points based on angle it makes to origin
         *
         * @param points points to be sorted
         * @param angle_offset offset added to each point's angle. Needed when
         * split at -pi/+pi boundary is undesirable.
         * @return PointCloud2D ordered points
         */
        static PointCloud2D orderPointsBasedOnAngle(
                PointCloud2D& points,
                float angle_offset = 0.0f);

        /**
         * @brief Calculate trajectory (vector of poses) for fixed velocity
         * using euler forward integration
         *
         * @param vel velocity at which the object is travelling
         * @param num_of_poses number of poses in trajectory
         * @param future_time time for which the object is travelling
         * @return std::vector<Pose2D> trajectory
         */
        static std::vector<Pose2D> calcTrajectory(
                const Velocity2D& vel,
                size_t num_of_poses,
                float future_time);

        /**
         * @brief Calculate the shortest angular difference between two given
         * angles. The result will always be between -pi and pi
         *
         * @param angle1 first angle
         * @param angle2 second angle
         * @return float shortest angle between first and second angle
         */
        static float calcShortestAngle(
                float angle1,
                float angle2);

        /**
         * @brief Calculate slope and constant of a line that is perpendicular
         * to the given line at a specific given point
         *
         * @param m slope of given line
         * @param c constant of given line
         * @param p point at which the given line and perpendicular intersect
         * @param perpendicular_m slope of perpendicular line
         * @param perpendicular_c constant of perpendicular line
         */
        static void findPerpendicularLineAt(
                float m,
                float c,
                const Point2D& p,
                float& perpendicular_m,
                float& perpendicular_c);

        /**
         * @brief Calculate squared perpendicular distance of a point from a
         * line.
         *
         * @param m slope of given line
         * @param c constant of given line
         * @param p point from which perpedicular distance needs to be
         * calculated
         * @return float squared perpendicular distance between point and line
         */
        static float calcSquaredDistToLine(
                float m,
                float c,
                const Point2D& p);

        /**
         * @brief Given a line and a point, calculate a projection point of the
         * given point on that line
         *
         * @param m slope of given line
         * @param c constant of given line
         * @param p point whose projection needs to be calculated
         * @return Point2D projected point
         */
        static Point2D calcProjectedPointOnLine(
                float m,
                float c,
                const Point2D& p);

        /**
         * @brief Given a line and a point, calculate a projection point of the
         * given point on that line. The line would be considered as a line
         * segment if `is_segment` is true, implying that the projected point
         * would be clipped to be within the line segment's start and end
         * points.
         * source: https://stackoverflow.com/a/1501725/10460994
         *
         * @param line_start a point on the given line (also treated as start
         * point if it is line segment)
         * @param line_end another point on the given line (also treated as end
         * point if it is line segment)
         * @param p point whose projection needs to be calculated
         * @param is_segment option to treat the line as a line segment or not.
         * True implies that the projected point would always be between
         * `line_start` and `line_end`.
         * @return Point2D projected point
         */
        static Point2D calcProjectedPointOnLine(
                const Point2D& line_start,
                const Point2D& line_end,
                const Point2D& p,
                bool is_segment);

        /**
         * @brief Given a line and a point, calculate a projection point of the
         * given point on either of the 2 axis on that line depending on the
         * slope of the line. If the given line is closer to X-axis in terms of
         * slope, then the projected point will have same x coordinate as the
         * given point and the y coordinate will be projected on that given
         * line. Similarly, if the given line is closer to Y-axis, then the
         * projected point's y coordinate will be same and x coordinate will be
         * projected.
         *
         * @param m slope of given line
         * @param c constant of given line
         * @param p point whose projection needs to be calculated
         * @return Point2D projected point
         */
        static Point2D calcProjectedPointOnMajorAxis(
                float m,
                float c,
                const Point2D& p);

        /**
         * @brief Find the distance of point p from a line formed by points a
         * and b
         *
         * @param a point on the given line (also treated as start
         * point if it is line segment)
         * @param b another point on the given line (also treated as end
         * point if it is line segment)
         * @param p point whose squared distance from the line needs to be
         * calculated
         * @param is_segment option to treat the line as a line segment or not.
         * \see Utils::calcProjectedPointOnLine
         * @return float squared distance between the given point and projected
         * point.
         */
        static float calcSquaredDistToLine(
                const Point2D& a,
                const Point2D& b,
                const Point2D& p,
                bool is_segment = false);

        /**
         * @brief 
         * 
         * @param pts 
         * @param start_index 
         * @param end_index 
         * @param m 
         * @param c 
         * @param delta 
         * @param itr_limit 
         * @return float 
         */
        static float fitLineRANSAC(
                const PointCloud2D& pts,
                unsigned start_index,
                unsigned end_index,
                float& m,
                float& c,
                float delta = 0.2f,
                size_t itr_limit = 10);

        /**
         * @brief 
         * 
         * @param pts 
         * @param m 
         * @param c 
         * @param delta 
         * @param itr_limit 
         * @return float 
         */
        static float fitLineRANSAC(
                const PointCloud2D& pts,
                float& m, float& c,
                float delta = 0.2f,
                size_t itr_limit = 10);

        /**
         * @brief 
         * 
         * @param pts 
         * @param start_index 
         * @param end_index 
         * @param line_segment 
         * @param delta 
         * @param itr_limit 
         * @return float 
         */
        static float fitLineSegmentRANSAC(
                const PointCloud2D& pts,
                unsigned start_index,
                unsigned end_index,
                LineSegment2D& line_segment,
                float delta = 0.2f,
                size_t itr_limit = 10);

        /**
         * @brief 
         * 
         * @param pts 
         * @param line_segment 
         * @param delta 
         * @param itr_limit 
         * @return float 
         */
        static float fitLineSegmentRANSAC(
                const PointCloud2D& pts,
                LineSegment2D& line_segment,
                float delta = 0.2f,
                size_t itr_limit = 10);

        /**
         * @brief 
         * 
         * @param pts 
         * @param score_threshold 
         * @param delta 
         * @param itr_limit 
         * @return std::vector<LineSegment2D> 
         */
        static std::vector<LineSegment2D> fitLineSegmentsRANSAC(
                const PointCloud2D& pts,
                float score_threshold = 0.9f,
                float delta = 0.2f,
                size_t itr_limit = 10);

        /**
         * @brief Fit a Circle from a subset of points using RANSAC
         *
         * @param pts entire pointcloud
         * @param start_index start of subset of points to consider
         * @param end_index end of subset of points to consider
         * @param circle fitted circle (output)
         * @param delta radius error
         * @param itr_limit number of iteration for random samples
         *
         * @return best score
         */
        static float fitCircleRANSAC(
                const PointCloud2D& pts,
                unsigned start_index,
                unsigned end_index,
                Circle& circle,
                float delta = 0.2f,
                size_t itr_limit = 10);

        /**
         * @brief Fit a Circle from a set of points using RANSAC
         *
         * @param pts entire pointcloud
         * @param circle fitted circle (output)
         * @param delta radius error
         * @param itr_limit number of iteration for random samples
         *
         * @return best score
         */
        static float fitCircleRANSAC(
                const PointCloud2D& pts,
                Circle& circle,
                float delta = 0.2f,
                size_t itr_limit = 10);

        /**
         * @brief 
         * 
         * @param pts 
         * @param start_index 
         * @param end_index 
         * @param line_segment 
         * @param is_ordered whether the points in pts are ordered in angle
         * @return float 
         */
        static float fitLineRegression(
                const PointCloud2D& pts,
                unsigned start_index,
                unsigned end_index,
                LineSegment2D& line_segment,
                bool is_ordered = false);

        /**
         * @brief 
         * 
         * @param pts 
         * @param line_segment 
         * @param is_ordered whether the points in pts are ordered in angle
         * @return float 
         */
        static float fitLineRegression(
                const PointCloud2D& pts,
                LineSegment2D& line_segment,
                bool is_ordered = false);

        /**
         * @brief 
         * 
         * @param pts 
         * @param error_threshold 
         * @return std::vector<LineSegment2D> 
         */
        static std::vector<LineSegment2D> applyPiecewiseRegression(
                const PointCloud2D& pts,
                float error_threshold = 0.1f);

        /**
         * @brief 
         * 
         * @param pts 
         * @param error_threshold 
         * @return std::vector<LineSegment2D> 
         */
        static std::vector<LineSegment2D> applyPiecewiseRegressionSplit(
                const PointCloud2D& pts,
                float error_threshold = 0.1f);

        /**
         * @brief 
         * 
         * @param line_segments 
         * @param distance_threshold 
         * @param angle_threshold 
         */
        static void mergeCloseLines(
                std::vector<LineSegment2D>& line_segments,
                float distance_threshold = 0.2f,
                float angle_threshold = 0.2f);

        /**
         * @brief 
         * 
         * @param line_segments 
         * @param distance_threshold 
         * @param angle_threshold 
         */
        static void mergeCloseLinesBF(
                std::vector<LineSegment2D>& line_segments,
                float distance_threshold = 0.2f,
                float angle_threshold = 0.2f);

        /**
         * @brief Merge co-linear line segments
         *
         * @param line_segments input line segments
         * @param distance_threshold threshold for distance between two line
         * segments
         * @param angle_threshold threshold for relative angle between two line
         * segments
         * @param perp_dist_threshold threshold for perpendicular distance
         * between two line segments
         */
        static void mergeCoLinearLines(
                std::vector<LineSegment2D>& line_segments,
                float distance_threshold = 0.2f,
                float angle_threshold = 0.2f,
                float perp_dist_threshold = 0.1f);

        /**
         * @brief 
         * 
         * @param pts 
         * @param regression_error_threshold 
         * @param distance_threshold 
         * @param angle_threshold 
         * @return std::vector<LineSegment2D> 
         */
        static std::vector<LineSegment2D> fitLineSegments(
                const std::vector<Point2D>& pts,
                float regression_error_threshold = 0.1f,
                float distance_threshold = 0.2,
                float angle_threshold = 0.2);

        /**
         * @brief 
         * 
         * @param value 
         * @param max_limit 
         * @param min_limit 
         * @return float 
         */
        static float clip(
                float value,
                float max_limit,
                float min_limit);

        /**
         * @brief clip `value` to be between `min_limit` and `max_limit` while
         * maintaining sign.\n
         * If `value` is positive, it clips in the range [min_limit, max_limit]
         * If `value` is negative, it clips in the range [-max_limit, -min_limit]
         *
         * @note: Both min_limit and max_limit needs to be positive values
         * 
         * @param value number to be clipped
         * @param max_limit positive maximum limit
         * @param min_limit positive minimum limit
         * @return float
         */
        static float clipSigned(
                float value,
                float max_limit,
                float min_limit);

        /**
         * @brief convert any angle value to be between -pi and pi
         * 
         * @param raw_angle an angular value in radians
         * @return float angular value in radians in range [-pi, pi]
         */
        static float clipAngle(
                float raw_angle);

        /**
         * @brief Clip XYTheta between max and min limits
         *
         * @param value raw value that needs to be clipped
         * @param max_limit higher than this will get clipped
         * @param min_limit lower than this will get clipped
         *
         * @return clipped XYTheta
         */
        static XYTheta clip(
                const XYTheta& value,
                const XYTheta& max_limit,
                const XYTheta& min_limit);

        /**
         * @brief 
         * 
         * @param vel 
         * @param max_vel 
         * @param min_vel 
         * @return Velocity2D 
         */
        static Velocity2D applyVelLimits(
                const Velocity2D& vel,
                const Velocity2D& max_vel,
                const Velocity2D& min_vel);

        /**
         * @brief calculate target velocity by ramping from curr_vel to
         * cmd_vel. Ramping is achieved by applying acceleration limits.
         * 
         * @param cmd_vel Target velocity
         * @param curr_vel current velocity
         * @param max_acc acceleration limits
         * @param delta_t time between now and next iteration
         * @return Velocity2D ramped velocity
         */
        static Velocity2D applyAccLimits(
                const Velocity2D& cmd_vel,
                const Velocity2D& curr_vel,
                const Acceleration2D& max_acc,
                float delta_t);

        /**
         * @brief 
         * 
         * @param src 
         * @param target 
         * @param t 
         * @return float 
         */
        static float applyLinearInterpolation(
                float src,
                float target,
                float t);


        /**
         * @brief Calculate an entire bezier spline curve full of points for
         * given control points.
         *
         * @param control_points control points to generate spline curve from
         * @param num_of_points number of points to populate the spline curve
         * with
         *
         * @return spline curve as a vector of points
         */
        static PointVec2D calcSplineCurvePoints(
                const PointVec2D& control_points,
                size_t num_of_points);

        /**
         * @brief Calculate the coefficients for a given row number in pascal's
         * triangle \n
         * for example, for row_num = 0, it would be    [1]
         *              for row_num = 1, it would be   [1 1]
         *              for row_num = 2, it would be  [1 2 1]
         *              for row_num = 3, it would be [1 3 3 1]
         *              ...
         *
         * @param row_num row number from top on pascal's triangle
         *
         * @return coefficients at the given row
         */
        static std::vector<unsigned int> calcPascalTriangleRowCoefficients(
                size_t row_num);

        /**
         * @brief Calculate a single spline curve point at given interpolation t
         * for given control points.
         *
         * @param control_points control points for bezier spline curve
         * @param coefficients pascal triangle coefficient for given curve
         * points
         * @param t interpolation factor (should be between 0.0 and 1.0)
         *
         * @return a point on spline curve at given interpolation t
         */
        static Point2D calcSplineCurvePoint(
                const PointVec2D& control_points,
                const std::vector<unsigned int>& coefficients,
                float t);


        /**
         * @brief Convert from PointCloud2D or PointCloud3D to
         * sensor_msgs::msg::PointCloud
         * 
         * @tparam T Point2D or Point3D
         * @param pc PointCloud2D or PointCloud3D
         * @param frame 
         * @return sensor_msgs::msg::PointCloud 
         */
        template <typename T>
        static sensor_msgs::msg::PointCloud convertToROSPointCloud(
                const std::vector<T>& pc,
                const std::string& frame = "base_link");

        /**
         * @brief 
         * 
         * @param pc 
         * @return PointCloud3D 
         */
        static PointCloud3D convertToPointCloud3D(
                const sensor_msgs::msg::PointCloud& pc);

        /**
         * @brief `row_sub_sample_factor` and `col_sub_sample_factor` parameterise how
         * many points are skipped. \n
         * e.g. if both their values is 1, no points are skipped \n
         *      if both their values are 2 and it is an organised cloud, the
         *      resulting cloud will be 1/4th the input cloud
         * 
         * @param cloud_msg 
         * @param row_sub_sample_factor 
         * @param col_sub_sample_factor 
         * @return PointCloud3D 
         */
        static PointCloud3D convertToPointCloud3D(
                const sensor_msgs::msg::PointCloud2& cloud_msg,
                size_t row_sub_sample_factor = 1,
                size_t col_sub_sample_factor = 1);

        /**
         * @brief Convert from LaserScan msg to PointCloud
         *
         * @tparam T Point2D or Point3D
         * @param scan laser scan message
         *
         * @return PointCloud2D or PointCloud3D
         */
        template <typename T>
        static std::vector<T> convertToPointCloud(
                const sensor_msgs::msg::LaserScan& scan);

        /**
         * @brief
         * 
         * @param angle 
         * @return float 
         */
        static float calcPerpendicularAngle(
                float angle);

        /**
         * @brief
         * 
         * @param angle 
         * @return float 
         */
        static float calcReverseAngle(
                float angle);

        /**
         * @brief 
         * 
         * @note Since the angles wrap around, the rule of thumb here is that the sector
         * considered is the one where you move counter-clockwise from min_angle and
         * all angles are always represented from -pi to pi
         * 
         * @param angle 
         * @param max_angle 
         * @param min_angle 
         * @return bool 
         */
        static bool isAngleWithinBounds(
                float angle,
                float max_angle,
                float min_angle);

        /**
         * @brief 
         * 
         * @param pose 
         * @param max_perp_dist 
         * @param step_size 
         * @return PointVec2D
         */
        static PointVec2D generatePerpendicularPointsAt(
                const Pose2D& pose,
                float max_perp_dist = 3.0f,
                float step_size = 0.1f);

        /**
         * @brief
         * 
         * @param a 
         * @param b 
         * @param c 
         * @return float 
         */
        static float calcAngleBetweenPoints(
                const Point2D& a,
                const Point2D& b,
                const Point2D& c);

        /**
         * @brief 
         * 
         * @param a 
         * @param b 
         * @param c 
         * @param tolerance 
         * @return WindingOrder 
         */
        static WindingOrder calcWindingOrder(
                const Point2D& a,
                const Point2D& b,
                const Point2D& c,
                float tolerance = 1e-6f);

        /**
         * @brief Convert from Quaternion to Euler angles
         *
         * @param qx
         * @param qy
         * @param qz
         * @param qw
         * @param roll rotation on X axis
         * @param pitch rotation on Y axis
         * @param yaw rotation on Z axis
         */
        static void convertQuaternionToEuler(
                float qx,
                float qy,
                float qz,
                float qw,
                float& roll,
                float& pitch,
                float& yaw);

        /**
         * @brief Convert from Euler to Quaternion angles
         *
         * @param roll rotation on X axis
         * @param pitch rotation on Y axis
         * @param yaw rotation on Z axis
         * @param qx
         * @param qy
         * @param qz
         * @param qw
         */
        static void convertEulerToQuaternion(
                float roll,
                float pitch,
                float yaw,
                float& qx,
                float& qy,
                float& qz,
                float& qw);

        /**
         * @brief
         * 
         * @param trajectory 
         * @param frame 
         * @return nav_msgs::msg::Path 
         */
        static nav_msgs::msg::Path convertToROSPath(
                const std::vector<Pose2D>& trajectory,
                const std::string& frame);

        /**
         * @brief
         * 
         * @param geometric_path 
         * @param frame 
         * @param red 
         * @param green 
         * @param blue 
         * @param alpha 
         * @param line_width 
         * @return visualization_msgs::msg::Marker 
         */
        static visualization_msgs::msg::Marker convertGeometricPathToMarker(
                const Path& geometric_path,
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float line_width = 0.05f);

        /**
         * @brief convert PointCloud2D or PointCloud3D to Marker
         *
         * @tparam T Point3D or Point3D
         * @param cloud PointCloud2D or PointCloud3D
         * @param frame
         * @param diameter
         * @param red
         * @param green
         * @param blue
         * @param alpha
         *
         * @return 
         */
        template <typename T>
        static visualization_msgs::msg::Marker convertPointCloudToMarker(
                const std::vector<T>& cloud,
                const std::string& frame = "base_link",
                float diameter = 0.05f,
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f);

        /**
         * @brief
         * 
         * @param string_label 
         * @param frame 
         * @param red 
         * @param green 
         * @param blue 
         * @param alpha 
         * @param size 
         * @return visualization_msgs::msg::Marker 
         */
        static visualization_msgs::msg::Marker convertStringToMarker(
                const std::string& string_label,
                const std::string& frame,
                float red = 0.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float size = 0.2f);
};

} // namespace geometry_common
} // namespace kelo
