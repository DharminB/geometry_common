/******************************************************************************
 * Copyright (c) 2022
 * KELO Robotics GmbH
 *
 * Author:
 * Dharmin Bakaraniya
 * Sushant Chavan
 * Leonardo Tan
 * Walter Nowak
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

#include <cmath>
#include <cstdlib>
#include <cassert>
#include <list>
#include <deque>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_common/TransformMatrix2D.h>
#include <geometry_common/Utils.h>

namespace kelo
{
namespace geometry_common
{

float Utils::roundFloat(
        float value,
        unsigned decimal_places)
{
    unsigned multiplier = std::pow(10, decimal_places);
    return ((float)((int)std::round(value * multiplier))) / multiplier;
}

template <typename T>
T Utils::calcMeanPoint(
        const std::vector<T>& points,
        unsigned start_index,
        unsigned end_index)
{
    T sum_pt;
    for ( size_t i = start_index; i <= end_index; i++ )
    {
        sum_pt = sum_pt + points[i];
    }
    return sum_pt / static_cast<float>(end_index - start_index + 1);
}
template Point2D Utils::calcMeanPoint(
        const std::vector<Point2D>& points,
        unsigned start_index,
        unsigned end_index);
template Point3D Utils::calcMeanPoint(
        const std::vector<Point3D>& points,
        unsigned start_index,
        unsigned end_index);

template <typename T>
T Utils::calcMeanPoint(
        const std::vector<T>& points)
{
    return Utils::calcMeanPoint(points, 0, points.size()-1);
}
template Point2D Utils::calcMeanPoint(const std::vector<Point2D>& points);
template Point3D Utils::calcMeanPoint(const std::vector<Point3D>& points);

template <typename T>
Pose2D Utils::calcMeanPose(
        const T& poses)
{
    Pose2D mean_cart_pose;
    if ( poses.size() == 0 )
    {
        return mean_cart_pose;
    }
    float cos_theta_sum = 0.0f;
    float sin_theta_sum = 0.0f;
    for ( const Pose2D& p : poses )
    {
        mean_cart_pose.x += p.x;
        mean_cart_pose.y += p.y;
        cos_theta_sum += std::cos(p.theta);
        sin_theta_sum += std::sin(p.theta);
    }
    mean_cart_pose.x /= poses.size();
    mean_cart_pose.y /= poses.size();
    mean_cart_pose.theta = std::atan2(sin_theta_sum/poses.size(),
                                      cos_theta_sum/poses.size());
    return mean_cart_pose;
}
template Pose2D Utils::calcMeanPose(const std::vector<Pose2D>& poses);
template Pose2D Utils::calcMeanPose(const std::deque<Pose2D>& poses);
template Pose2D Utils::calcMeanPose(const std::list<Pose2D>& poses);

template <typename T>
T Utils::calcClosestPoint(
        const std::vector<T>& points,
        const T& pt)
{
    size_t min_pt_index = 0;
    float min_dist_sq = std::numeric_limits<float>::max();
    for ( size_t i = 0; i < points.size(); i++ )
    {
        float dist_sq = points[i].squaredDistTo(pt);
        if ( dist_sq < min_dist_sq )
        {
            min_pt_index = i;
            min_dist_sq = dist_sq;
        }
    }
    return points[min_pt_index];
}
template Point2D Utils::calcClosestPoint(
        const std::vector<Point2D>& points,
        const Point2D& pt);
template Point3D Utils::calcClosestPoint(
        const std::vector<Point3D>& points,
        const Point3D& pt);

std::vector<PointCloud2D> Utils::clusterPoints(
        const PointCloud2D& points,
        float cluster_distance_threshold,
        size_t min_cluster_size)
{
    std::vector<PointCloud2D> clusters;

    /* populate remaining_points */
    std::list<Point2D> remaining_points;
    for ( const Point2D& p : points )
    {
        remaining_points.push_back(Point2D(p));
    }

    float threshold_dist_sq = std::pow(cluster_distance_threshold, 2);

    /* cluster remaining_points iteratively */
    while ( !remaining_points.empty() )
    {
        std::vector<Point2D> cluster;
        std::list<Point2D> fringe;
        fringe.push_back(remaining_points.front());
        remaining_points.pop_front();

        while ( !fringe.empty() )
        {
            // get first point from fringe
            Point2D point = fringe.front();
            fringe.pop_front();
            cluster.push_back(point);

            // iterate over remaining points
            auto pt = remaining_points.begin();
            while ( pt != remaining_points.end() )
            {
                if ( point.squaredDistTo(*pt) < threshold_dist_sq )
                {
                    fringe.push_back(*pt);
                    pt = remaining_points.erase(pt);
                    continue;
                }
                ++pt;
            }
        }
        if ( cluster.size() > min_cluster_size )
        {
            clusters.push_back(cluster);
        }
    }
    return clusters;
}

std::vector<PointCloud2D> Utils::clusterOrderedPoints(
        const PointCloud2D& points,
        float cluster_distance_threshold,
        size_t min_cluster_size)
{
    std::vector<PointCloud2D> clusters;

    /* populate remaining_points */
    std::list<Point2D> remaining_points;
    for ( Point2D p : points )
    {
        remaining_points.push_back(Point2D(p));
    }

    float threshold_dist_sq = std::pow(cluster_distance_threshold, 2);

    while ( !remaining_points.empty() )
    {
        std::vector<Point2D> cluster;
        cluster.push_back(remaining_points.front());
        remaining_points.pop_front();

        // iterate over remaining points
        auto pt = remaining_points.begin();
        while ( pt != remaining_points.end() )
        {
            if ( cluster.back().squaredDistTo(*pt) < threshold_dist_sq )
            {
                cluster.push_back(*pt);
                pt = remaining_points.erase(pt);
                continue;
            }
            ++pt;
        }
        if ( cluster.size() > min_cluster_size )
        {
            clusters.push_back(cluster);
        }
    }

    /* for 360 degree laser points */
    if ( clusters.size() > 1 &&
         clusters.front().front().squaredDistTo(clusters.back().back()) < threshold_dist_sq )
    {
        clusters.front().reserve(clusters.front().size() + clusters.back().size());
        clusters.front().insert(clusters.front().begin(), clusters.back().begin(),
                                clusters.back().end());
        clusters.pop_back();
    }
    return clusters;
}

std::vector<Pose2D> Utils::calcTrajectory(
        const Velocity2D& vel,
        size_t num_of_poses,
        float future_time)
{
    std::vector<Pose2D> traj;
    traj.reserve(num_of_poses+1);

    float delta_t = future_time/num_of_poses;

    // d = v * t
    TransformMatrix2D vel_tf_mat(vel * delta_t);
    TransformMatrix2D pos_mat; // identity mat

    /* add current pose (for extra safety) */
    traj.push_back(Pose2D());

    for ( size_t i = 0; i < num_of_poses; i++ )
    {
        pos_mat *= vel_tf_mat;
        traj.push_back(pos_mat.asPose2D());
    }
    return traj;
}

float Utils::calcShortestAngle(
        float angle1,
        float angle2)
{
    return std::atan2(std::sin(angle1 - angle2), std::cos(angle1 - angle2));
}

void Utils::findPerpendicularLineAt(
        float m,
        float c,
        const Point2D& p,
        float& perpendicular_m,
        float& perpendicular_c)
{
    perpendicular_m = ( std::fabs(m) < 1e-8f ) ? 1e8f : -1/m;
    perpendicular_c = p.y - (perpendicular_m * p.x);
}

float Utils::calcSquaredDistToLine(
        float m,
        float c,
        const Point2D& p)
{
    Point2D proj_pt = Utils::calcProjectedPointOnLine(m, c, p);
    return p.squaredDistTo(proj_pt);
}

Point2D Utils::calcProjectedPointOnLine(
        float m,
        float c,
        const Point2D& p)
{
    float perpendicular_m, perpendicular_c;
    Utils::findPerpendicularLineAt(m, c, p, perpendicular_m, perpendicular_c);
    Point2D proj_pt;
    proj_pt.x = (perpendicular_c - c) / (m - perpendicular_m);
    proj_pt.y = (m * proj_pt.x) + c;
    return proj_pt;
}

Point2D Utils::calcProjectedPointOnLine(
        const Point2D& line_start,
        const Point2D& line_end,
        const Point2D& p,
        bool is_segment)
{
    Point2D proj_pt;
    float length_sq = line_start.squaredDistTo(line_end);
    if ( length_sq < 1e-10f ) // check == 0
    {
        proj_pt = Point2D(line_start);
        return proj_pt;
    }
    Vector2D p_vec = p - line_start;
    Vector2D line_vec = line_end - line_start;
    float t = p_vec.dotProduct(line_vec) / length_sq;
    if ( is_segment )
    {
        t = Utils::clip(t, 1.0f, 0.0f);
    }
    return line_start + (line_vec * t);
}

Point2D Utils::calcProjectedPointOnMajorAxis(
        float m,
        float c,
        const Point2D& p)
{
    Point2D proj_pt;
    bool major_axis_x = ( std::fabs(m) < 1.0f );
    if ( major_axis_x )
    {
        proj_pt.x = p.x;
        proj_pt.y = (m * p.x) + c;
    }
    else
    {
        proj_pt.y = p.y;
        proj_pt.x = (p.y - c) / m;
    }
    return proj_pt;
}

float Utils::calcSquaredDistToLine(
        const Point2D& a,
        const Point2D& b,
        const Point2D& p,
        bool is_segment)
{
    return p.squaredDistTo(Utils::calcProjectedPointOnLine(a, b, p, is_segment));
}

float Utils::fitLineRANSAC(
        const PointCloud2D& pts,
        unsigned start_index,
        unsigned end_index,
        float& m,
        float& c,
        float delta,
        size_t itr_limit)
{
    if ( end_index <= start_index )
    {
        m = 0.0f;
        c = 0.0f;
        return 0.0f;
    }

    unsigned max_score = 0;
    size_t index_1 = start_index;
    size_t index_2 = end_index;
    size_t num_of_points = end_index-start_index+1;
    float delta_sq = delta * delta;
    for ( size_t itr_num = 0; itr_num < itr_limit; ++itr_num )
    {
        size_t ind_1 = (std::rand() % num_of_points) + start_index;
        size_t ind_2 = (std::rand() % num_of_points) + start_index;
        unsigned score = 0;
        for ( size_t i = start_index; i <= end_index; i++ )
        {
            if ( Utils::calcSquaredDistToLine(
                        pts[ind_1], pts[ind_2], pts[i]) < delta_sq )
            {
                score ++;
            }
        }
        if ( score > max_score )
        {
            max_score = score;
            index_1 = ind_1;
            index_2 = ind_2;
        }
    }

    float dx = pts[index_1].x - pts[index_2].x;
    if ( std::fabs(dx) < 1e-8 )
    {
        dx = 1e-8;
    }
    m = (pts[index_1].y - pts[index_2].y) / dx;
    c = pts[index_1].y - m*pts[index_1].x;
    return (float)max_score/num_of_points;
}

float Utils::fitLineRANSAC(
        const PointCloud2D& pts,
        float& m,
        float& c,
        float delta,
        size_t itr_limit)
{
    return Utils::fitLineRANSAC(pts, 0, pts.size()-1, m, c, delta, itr_limit);
}

float Utils::fitLineSegmentRANSAC(
        const PointCloud2D& pts,
        unsigned start_index,
        unsigned end_index,
        LineSegment2D& line_segment,
        float delta,
        size_t itr_limit)
{
    float m, c, score;
    float delta_sq = delta * delta;
    score = Utils::fitLineRANSAC(pts, start_index, end_index, m, c, delta, itr_limit);
    line_segment.start.x = 1e6f;
    line_segment.start.y = 1e6f;
    line_segment.end.x = -1e6f;
    line_segment.end.y = -1e6f;
    bool major_axis_x = ( std::fabs(m) < 1.0f );
    for ( size_t i = start_index; i <= end_index; i++ )
    {
        if ( Utils::calcSquaredDistToLine(m, c, pts[i]) < delta_sq )
        {
            if ( major_axis_x )
            {
                if ( pts[i].x < line_segment.start.x )
                {
                    line_segment.start = Utils::calcProjectedPointOnMajorAxis(m, c, pts[i]);
                }
                if ( pts[i].x > line_segment.end.x )
                {
                    line_segment.end = Utils::calcProjectedPointOnMajorAxis(m, c, pts[i]);
                }
            }
            else
            {
                if ( pts[i].y < line_segment.start.y )
                {
                    line_segment.start = Utils::calcProjectedPointOnMajorAxis(m, c, pts[i]);
                }
                if ( pts[i].y > line_segment.end.y )
                {
                    line_segment.end = Utils::calcProjectedPointOnMajorAxis(m, c, pts[i]);
                }
            }
        }
    }
    return score;
}

float Utils::fitLineSegmentRANSAC(
        const PointCloud2D& pts,
        LineSegment2D& line_segment,
        float delta,
        size_t itr_limit)
{
    return Utils::fitLineSegmentRANSAC(pts, 0, pts.size()-1, line_segment, delta, itr_limit);
}

std::vector<LineSegment2D> Utils::fitLineSegmentsRANSAC(
        const PointCloud2D& pts,
        float score_threshold,
        float delta,
        size_t itr_limit)
{
    struct RegressionLineSegment
    {
        unsigned start_index, end_index;
        LineSegment2D line_segment;
    };

    std::vector<LineSegment2D> line_segments;
    if (pts.size() < 2)
    {
        return line_segments;
    }

    /* fill in the initial segments */
    std::vector<RegressionLineSegment> segments(1);
    segments[0].start_index = 0;
    segments[0].end_index = pts.size()-1;

    float score = Utils::fitLineSegmentRANSAC(pts, segments[0].start_index,
                                              segments[0].end_index, 
                                              segments[0].line_segment,
                                              delta, itr_limit);
    if ( score > score_threshold )
    {
        line_segments.push_back(segments[0].line_segment);
        return line_segments;
    }

    std::vector<float> scores;
    scores.push_back(score);

    while ( true )
    {
        /* find segment with minimum error */
        size_t i = 0;
        float lowest_score = 0.0f;
        for ( size_t j = 0; j < scores.size(); j++ )
        {
            float score = scores[j];
            if ( score < lowest_score )
            {
                lowest_score = score;
                i = j;
            }
        }

        /* if segment with lowest score is within score_threshold, stop splitting */
        if ( lowest_score > score_threshold )
        {
            break;
        }

        /* if segment with lowest_score is only 2 points, stop splitting */
        if ( segments[i].end_index - segments[i].start_index < 3 )
        {
            break;
        }

        /* split the worst segments and update scores */
        size_t split_pt_index = segments[i].start_index;
        float max_dist = 0.0f;
        for ( size_t j = segments[i].start_index+1; j+1 <= segments[i].end_index; j++ )
        {
            // float dist = Utils::distToLineSegmentSquared(segments[i].line_segment, pts[j]);
            float dist = Utils::calcSquaredDistToLine(
                    pts[segments[i].start_index],
                    pts[segments[i].end_index],
                    pts[j], true);
            // std::cout << j << " " << dist << std::endl;
            if ( dist > max_dist )
            {
                max_dist = dist;
                split_pt_index = j;
            }
        }
        RegressionLineSegment new_segment;
        new_segment.start_index = segments[i].start_index;
        new_segment.end_index = split_pt_index;
        segments[i].start_index = split_pt_index+1;
        segments.insert(segments.begin() + i, new_segment);
        scores.insert(scores.begin() + i, 0.0f);

        scores[i] = Utils::fitLineSegmentRANSAC(pts, segments[i].start_index,
                                                segments[i].end_index,
                                                segments[i].line_segment,
                                                delta, itr_limit);
        scores[i+1] = Utils::fitLineSegmentRANSAC(pts, segments[i+1].start_index,
                                                  segments[i+1].end_index,
                                                  segments[i+1].line_segment,
                                                  delta, itr_limit);
    }

    line_segments.clear();
    line_segments.reserve(segments.size());
    /* create line_segments from regression line segments */
    for ( RegressionLineSegment rls : segments )
    {
        line_segments.push_back(rls.line_segment);
    }
    return line_segments;
}

float Utils::fitCircleRANSAC(
        const PointCloud2D& pts,
        unsigned start_index,
        unsigned end_index,
        Circle& circle,
        float delta,
        size_t itr_limit)
{
    if ( end_index <= start_index+1 )
    {
        circle.x = 0.0f;
        circle.y = 0.0f;
        circle.r = 0.0f;
        return 0.0f;
    }

    unsigned max_score = 0;
    size_t num_of_points = end_index-start_index+1;
    Circle candidate_circle;
    for ( size_t itr_num = 0; itr_num < itr_limit; ++itr_num )
    {
        size_t ind_1 = (std::rand() % num_of_points) + start_index;
        size_t ind_2 = ind_1;
        while ( ind_2 == ind_1 )
        {
            ind_2 = (std::rand() % num_of_points) + start_index;
        }
        size_t ind_3 = ind_1;
        while ( ind_3 == ind_1 || ind_3 == ind_2 )
        {
            ind_3 = (std::rand() % num_of_points) + start_index;
        }
        if ( !Circle::fromPoints(pts[ind_1], pts[ind_2], pts[ind_3], candidate_circle) )
        {
            continue;
        }

        unsigned score = 0;
        for ( size_t i = start_index; i <= end_index; i++ )
        {
            if ( std::fabs(pts[i].distTo(candidate_circle) - candidate_circle.r) < delta )
            {
                score ++;
            }
        }
        if ( score > max_score )
        {
            max_score = score;
            circle = candidate_circle;
        }
    }

    return (float)max_score/num_of_points;
}

float Utils::fitCircleRANSAC(
        const PointCloud2D& pts,
        Circle& circle,
        float delta,
        size_t itr_limit)
{
    return Utils::fitCircleRANSAC(pts, 0, pts.size()-1, circle, delta, itr_limit);
}

float Utils::fitLineRegression(
        const PointCloud2D& pts,
        unsigned start_index,
        unsigned end_index,
        LineSegment2D& line_segment,
        bool is_ordered)
{
    if ( pts.size() < 2 ||
         start_index >= pts.size() || end_index >= pts.size() ||
         end_index <= start_index || end_index - start_index < 2 )
    {
        line_segment = LineSegment2D();
        return 0.0f;
    }
    Point2D mean_pt = Utils::calcMeanPoint(pts, start_index, end_index);

    Point2D start_pt, end_pt;
    if ( is_ordered )
    {
        start_pt = pts[start_index];
        end_pt = pts[end_index];
    }
    else
    {
        end_pt.x = std::numeric_limits<float>::lowest();
        end_pt.y = std::numeric_limits<float>::lowest();
        start_pt.x = std::numeric_limits<float>::max();
        start_pt.y = std::numeric_limits<float>::max();
        for ( size_t i = start_index; i <= end_index; i++ )
        {
            if ( pts[i].x < start_pt.x )
            {
                start_pt.x = pts[i].x;
            }
            if ( pts[i].y < start_pt.y )
            {
                start_pt.y = pts[i].y;
            }
            if ( pts[i].x > end_pt.x )
            {
                end_pt.x = pts[i].x;
            }
            if ( pts[i].y > end_pt.y )
            {
                end_pt.y = pts[i].y;
            }
        }
    }
    Point2D diff = end_pt - start_pt;
    bool swap_axis = ( std::fabs(diff.x) < std::fabs(diff.y) );
    float numerator = 0.0f;
    float denominator = 0.0f;
    for ( size_t i = start_index; i <= end_index; i++ )
    {
        numerator += (pts[i].x - mean_pt.x) * (pts[i].y - mean_pt.y);
        if ( !swap_axis )
        {
            denominator += std::pow(pts[i].x - mean_pt.x, 2);
        }
        else
        {
            denominator += std::pow(pts[i].y - mean_pt.y, 2);
        }
    }
    if ( denominator < 1e-8f )
    {
        denominator = 1e-8f;
    }

    if ( !swap_axis )
    {
        float m = numerator / denominator;
        float c = mean_pt.y - (m * mean_pt.x);
        line_segment.start.x = start_pt.x;
        line_segment.start.y = (m * start_pt.x) + c;
        line_segment.end.x = end_pt.x;
        line_segment.end.y = (m * end_pt.x) + c;
    }
    else
    {
        float n = numerator / denominator;
        float d = mean_pt.x - (n * mean_pt.y);
        line_segment.start.y = start_pt.y;
        line_segment.start.x = (n * start_pt.y) + d;
        line_segment.end.y = end_pt.y;
        line_segment.end.x = (n * end_pt.y) + d;
    }

    float error = 0.0f;
    for ( size_t i = start_index; i <= end_index; i++ )
    {
        error += line_segment.squaredMinDistTo(pts[i]);
    }
    return error;
}

float Utils::fitLineRegression(
        const PointCloud2D& pts,
        LineSegment2D& line_segment,
        bool is_ordered)
{
    return Utils::fitLineRegression(pts, 0, pts.size()-1, line_segment, is_ordered);
}

std::vector<LineSegment2D> Utils::applyPiecewiseRegression(
        const PointCloud2D& pts,
        float error_threshold)
{
    struct RegressionLineSegment
    {
        unsigned start_index, end_index;
    };

    std::vector<LineSegment2D> line_segments;
    if (pts.size() < 2)
    {
        return line_segments;
    }

    /* fill in the initial segments */
    std::vector<RegressionLineSegment> segments(pts.size() / 2);
    for ( size_t i = 0; i < segments.size(); i++ )
    {
        segments[i].start_index = 2*i;
        segments[i].end_index = 2*i + 1;
        /* if pts has odd num of points, last segment should have 3 points */
        if ( (2*i)+3 >= pts.size() )
        {
            segments[i].end_index = pts.size() - 1;
        }
    }

    /* errors when 2 consecutive segments are merged */
    std::vector<float> errors(segments.size()-1);
    LineSegment2D line_segment; // not used;
    for ( size_t i = 0; i < errors.size(); i++ )
    {
        errors[i] = Utils::fitLineRegression(
                pts, segments[i].start_index, segments[i+1].end_index,
                line_segment, true);
    }

    while ( segments.size() > 1 )
    {
        /* find consecutive segments with minimum error if merged */
        size_t i = 0;
        float lowest_error = 1e6f; // just a large number
        for ( size_t j = 0; j < errors.size(); j++ )
        {
            float error = errors[j];
            if ( error < lowest_error )
            {
                lowest_error = error;
                i = j;
            }
        }

        /* if merged_segment with least error violates error_threshold, stop merging */
        if ( lowest_error > error_threshold )
        {
            break;
        }

        /* merge the best segments and update errors */
        segments[i].end_index = segments[i+1].end_index;
        segments.erase(segments.begin() + i + 1);

        LineSegment2D line_segment; // not used
        if (i > 0)
        {
            errors[i-1] = Utils::fitLineRegression(
                    pts, segments[i-1].start_index, segments[i].end_index,
                    line_segment, true);
        }
        if (i < segments.size() - 1)
        {
            errors[i+1] = Utils::fitLineRegression(
                    pts, segments[i].start_index, segments[i+1].end_index,
                    line_segment, true);
        }
        errors.erase(errors.begin() + i);
    }

    /* create line_segments from regression line segments */
    for ( RegressionLineSegment rls : segments )
    {
        LineSegment2D l;
        Utils::fitLineRegression(pts, rls.start_index, rls.end_index, l, true);
        line_segments.push_back(l);
    }
    return line_segments;
}

std::vector<LineSegment2D> Utils::applyPiecewiseRegressionSplit(
        const PointCloud2D& pts,
        float error_threshold)
{
    struct RegressionLineSegment
    {
        unsigned start_index, end_index;
        LineSegment2D line_segment;
    };

    std::vector<LineSegment2D> line_segments;
    if (pts.size() < 2)
    {
        return line_segments;
    }

    /* fill in the initial segments */
    std::vector<RegressionLineSegment> segments(1);
    segments[0].start_index = 0;
    segments[0].end_index = pts.size()-1;

    float error = Utils::fitLineRegression(
            pts, segments[0].start_index, segments[0].end_index,
            segments[0].line_segment, true);
    if ( error < error_threshold )
    {
        line_segments.push_back(segments[0].line_segment);
        return line_segments;
    }

    std::vector<float> errors;
    errors.push_back(error);

    while ( true )
    {
        /* find segment with maximum error */
        size_t i = 0;
        float highest_error = 0.0f;
        for ( size_t j = 0; j < errors.size(); j++ )
        {
            float error = errors[j];
            if ( error > highest_error )
            {
                highest_error = error;
                i = j;
            }
        }

        /* if segment with highest error is within error_threshold, stop splitting */
        if ( highest_error < error_threshold )
        {
            break;
        }

        /* if segment with highest error is only 2 points, stop splitting */
        if ( segments[i].end_index - segments[i].start_index < 3 )
        {
            break;
        }

        /* split the worst segments and update errors */
        size_t split_pt_index = segments[i].start_index;
        float max_dist = 0.0f;
        for ( size_t j = segments[i].start_index+1; j+1 <= segments[i].end_index; j++ )
        {
            // float dist = Utils::distToLineSegmentSquared(segments[i].line_segment,
            //                                              pts[j]);
            float dist = Utils::calcSquaredDistToLine(pts[segments[i].start_index],
                                                      pts[segments[i].end_index],
                                                      pts[j], true);
            if ( dist > max_dist )
            {
                max_dist = dist;
                split_pt_index = j;
            }
        }
        RegressionLineSegment new_segment;
        new_segment.start_index = segments[i].start_index;
        new_segment.end_index = split_pt_index;
        segments[i].start_index = split_pt_index+1;
        segments.insert(segments.begin() + i, new_segment);
        errors.insert(errors.begin() + i, 0.0f);

        errors[i] = Utils::fitLineRegression(
                pts, segments[i].start_index, segments[i].end_index,
                segments[i].line_segment, true);
        errors[i+1] = Utils::fitLineRegression(
                pts, segments[i+1].start_index, segments[i+1].end_index,
                segments[i+1].line_segment, true);
    }

    /* create line_segments from regression line segments */
    for ( RegressionLineSegment rls : segments )
    {
        line_segments.push_back(rls.line_segment);
    }
    return line_segments;
}

void Utils::mergeCloseLines(
        std::vector<LineSegment2D>& line_segments,
        float distance_threshold,
        float angle_threshold)
{
    if (line_segments.size() < 2)
    {
        return;
    }

    size_t i = 0;

    while ( i < line_segments.size()-1 )
    {
        float linear_dist = line_segments[i].end.distTo(line_segments[i+1].start);
        float angular_dist = Utils::calcShortestAngle(line_segments[i].angle(),
                                                      line_segments[i+1].angle());
        if ( linear_dist < distance_threshold &&
             std::fabs(angular_dist) < angle_threshold )
        {
            line_segments[i].end = line_segments[i+1].end;
            line_segments.erase(line_segments.begin() + i + 1);
            continue;
        }
        i++;
    }
}

void Utils::mergeCloseLinesBF(
        std::vector<LineSegment2D>& line_segments,
        float distance_threshold,
        float angle_threshold)
{
    if (line_segments.size() < 2)
    {
        return;
    }

    size_t skip_index = 1;

    while ( skip_index < line_segments.size() )
    {
        size_t i = 0;
        while ( i+skip_index < line_segments.size() )
        {
            float linear_dist = line_segments[i].end.distTo(
                    line_segments[i+skip_index].start);
            float angular_dist = Utils::calcShortestAngle(
                    line_segments[i].angle(), line_segments[i+skip_index].angle());
            if ( linear_dist < distance_threshold &&
                 std::fabs(angular_dist) < angle_threshold )
            {
                line_segments[i].end = line_segments[i+skip_index].end;
                line_segments.erase(line_segments.begin() + i + skip_index);
                continue;
            }
            i++;
        }
        skip_index ++;
    }
}

void Utils::mergeCoLinearLines(
        std::vector<LineSegment2D>& line_segments,
        float perp_dist_threshold)
{
    if (line_segments.size() < 2)
    {
        return;
    }

    size_t skip_index = 1;

    while ( skip_index < line_segments.size() )
    {
        size_t i = 0;
        while ( i+skip_index < line_segments.size() )
        {
            const Point2D start_proj_pt = Utils::calcProjectedPointOnLine(
                    line_segments[i].start, line_segments[i].end,
                    line_segments[i+skip_index].start, false);
            float start_perp_dist = start_proj_pt.distTo(
                    line_segments[i+skip_index].start);
            const Point2D end_proj_pt = Utils::calcProjectedPointOnLine(
                    line_segments[i].start, line_segments[i].end,
                    line_segments[i+skip_index].end, false);
            float end_perp_dist = end_proj_pt.distTo(
                    line_segments[i+skip_index].end);
            if ( start_perp_dist < perp_dist_threshold &&
                 end_perp_dist < perp_dist_threshold )
            {
                line_segments[i].end = line_segments[i+skip_index].end;
                line_segments.erase(line_segments.begin() + i + skip_index);
                continue;
            }
            i++;
        }
        skip_index ++;
    }
}

std::vector<LineSegment2D> Utils::fitLineSegments(
        const PointCloud2D& pts,
        float regression_error_threshold,
        float distance_threshold,
        float angle_threshold)
{
    std::vector<LineSegment2D> lines = Utils::applyPiecewiseRegression(
            pts, regression_error_threshold);
    Utils::mergeCloseLines(lines, distance_threshold, angle_threshold);
    return lines;
}

float Utils::clip(
        float value,
        float max_limit,
        float min_limit)
{
    return std::max(std::min(value, max_limit), min_limit);
}

float Utils::clipSigned(
        float value,
        float max_limit,
        float min_limit)
{
    return std::copysign(Utils::clip(std::fabs(value), max_limit, min_limit), value);
}

float Utils::clipAngle(
        float raw_angle)
{
    float two_pi = 2.0f * M_PI;
    float angle = ( std::fabs(raw_angle) > two_pi )
                  ? raw_angle - (std::floor(raw_angle/two_pi) * two_pi)
                  : raw_angle;
    if ( angle > M_PI )
    {
        angle -= two_pi;
    }
    else if ( angle < -M_PI )
    {
        angle += two_pi;
    }
    return angle;
}

XYTheta Utils::clip(
        const XYTheta& value,
        const XYTheta& max_limit,
        const XYTheta& min_limit)
{
    XYTheta clipped_xytheta;
    clipped_xytheta.x = Utils::clip(value.x, max_limit.x, min_limit.x);
    clipped_xytheta.y = Utils::clip(value.y, max_limit.y, min_limit.y);
    clipped_xytheta.theta = Utils::clip(value.theta, max_limit.theta, min_limit.theta);
    return clipped_xytheta;
}

Velocity2D Utils::applyVelLimits(
        const Velocity2D& vel,
        const Velocity2D& max_vel,
        const Velocity2D& min_vel)
{
    return Utils::clip(vel, max_vel, min_vel);
}

Velocity2D Utils::applyAccLimits(
        const Velocity2D& cmd_vel,
        const Velocity2D& curr_vel,
        const Acceleration2D& max_acc,
        float delta_t)
{
    Velocity2D vel;
    Acceleration2D max_acc_per_loop = max_acc * delta_t;
    return Utils::clip(cmd_vel, curr_vel + max_acc_per_loop, curr_vel - max_acc_per_loop);
}


float Utils::applyLinearInterpolation(
        float src,
        float target,
        float t)
{
    return ( t >= 1.0f ) ? target :
           ( t <= 0.0f ) ? src    : (src * (1.0f - t)) + (target * t);
}

PointVec2D Utils::calcSplineCurvePoints(
        const PointVec2D& control_points,
        size_t num_of_points)
{
    PointVec2D curve_points;
    if ( control_points.size() < 2 || num_of_points < 2 )
    {
        return curve_points;
    }

    size_t order = control_points.size() - 1;
    std::vector<unsigned int> coef = Utils::calcPascalTriangleRowCoefficients(order);
    float offset = 1.0f / (num_of_points-1);
    curve_points.reserve(num_of_points);
    curve_points.push_back(control_points[0]); // add start point as first curve point
    // add n-2 curve points in middle
    for ( size_t factor = 1; factor+1 < num_of_points; factor++ )
    {
        float t = offset * factor;
        curve_points.push_back(Utils::calcSplineCurvePoint(control_points, coef, t));
    }
    curve_points.push_back(control_points.back()); // add end point as last curve point
    return curve_points;
}

std::vector<unsigned int> Utils::calcPascalTriangleRowCoefficients(
        size_t row_num)
{
    std::vector<unsigned int> coefficents;
    coefficents.reserve(row_num+1);
    coefficents.push_back(1);
    for ( size_t i = 1; i < row_num+1; ++i )
    {
        coefficents.push_back( (coefficents.back() * (row_num + 1 - i)) / i );
    }
    return coefficents;
}

Point2D Utils::calcSplineCurvePoint(
        const PointVec2D& control_points,
        const std::vector<unsigned int>& coefficients,
        float t)
{
    Point2D curve_point;
    if ( control_points.size() < 2 )
    {
        return control_points.front();
    }

    size_t order = control_points.size() - 1;
    for ( size_t i = 0; i < order+1; i++ )
    {
        curve_point.x += static_cast<float>(coefficients[i])
                       * pow(1.0f-t, order-i)
                       * pow(t, i)
                       * control_points[i].x;
        curve_point.y += static_cast<float>(coefficients[i])
                       * pow(1.0f-t, order-i)
                       * pow(t, i)
                       * control_points[i].y;
    }
    return curve_point;
}

template <typename T>
sensor_msgs::PointCloud Utils::convertToROSPointCloud(
        const std::vector<T>& pc,
        const std::string& frame)
{
    sensor_msgs::PointCloud cloud;
    // cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = frame;
    cloud.points.reserve(pc.size());
    for ( const T& pt : pc )
    {
        cloud.points.push_back(pt.asPoint32());
    }
    return cloud;
}
template sensor_msgs::PointCloud Utils::convertToROSPointCloud(
        const PointCloud2D& pc, const std::string& frame);
template sensor_msgs::PointCloud Utils::convertToROSPointCloud(
        const PointCloud3D& pc, const std::string& frame);

PointCloud3D Utils::convertToPointCloud3D(
        const sensor_msgs::PointCloud& pc)
{
    std::vector<Point3D> points;
    points.reserve(pc.points.size());
    for ( const geometry_msgs::Point32& p : pc.points )
    {
        points.push_back(Point3D(p));
    }
    return points;
}

PointCloud3D Utils::convertToPointCloud3D(
        const sensor_msgs::PointCloud2& cloud_msg,
        size_t row_sub_sample_factor,
        size_t col_sub_sample_factor)
{
    if ( cloud_msg.height == 0 || cloud_msg.width == 0 )
    {
        return std::vector<Point3D>();
    }
    PointCloud3D points;
    if ( cloud_msg.height == 1 ) // unorganised cloud
    {
        row_sub_sample_factor = 1;
    }
    points.reserve((cloud_msg.height / row_sub_sample_factor) *
                   (cloud_msg.width / col_sub_sample_factor));
    size_t col = 0, row = 0;
    size_t col_remainder = cloud_msg.width % col_sub_sample_factor;
    size_t row_skip_factor = (cloud_msg.width * (row_sub_sample_factor-1)) + col_remainder;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_msg, "z");
    while ( iter_x != iter_x.end() && row < cloud_msg.height )
    {
        if ( !std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z) )
        {
            points.push_back(Point3D(*iter_x, *iter_y, *iter_z));
        }

        col += col_sub_sample_factor;
        if ( col >= cloud_msg.width )
        {
            col = 0;
            row += row_sub_sample_factor;
            iter_x += row_skip_factor;
            iter_y += row_skip_factor;
            iter_z += row_skip_factor;
            continue;
        }
        iter_x += col_sub_sample_factor;
        iter_y += col_sub_sample_factor;
        iter_z += col_sub_sample_factor;
    }
    return points;
}

template <typename T>
std::vector<T> Utils::convertToPointCloud(
        const sensor_msgs::LaserScan& scan)
{
    std::vector<T> laser_pts;
    for ( size_t i = 0; i < scan.ranges.size(); i++ )
    {
        if ( std::isnan(scan.ranges[i]) ||
             std::isinf(scan.ranges[i]) ||
             scan.ranges[i] >= scan.range_max ||
             scan.ranges[i] <= scan.range_min )
        {
            continue;
        }
        float angle = scan.angle_min + (i * scan.angle_increment);
        laser_pts.push_back(T(scan.ranges[i] * std::cos(angle),
                              scan.ranges[i] * std::sin(angle)));
    }
    return laser_pts;
}
template PointCloud2D Utils::convertToPointCloud<Point2D>(
        const sensor_msgs::LaserScan& scan);
template PointCloud3D Utils::convertToPointCloud<Point3D>(
        const sensor_msgs::LaserScan& scan);

float Utils::calcPerpendicularAngle(
        float angle)
{
    float perpendicular_angle = angle + M_PI/2;
    if ( perpendicular_angle > M_PI )
    {
        perpendicular_angle -= 2*M_PI;
    }
    return perpendicular_angle;
}

float Utils::calcReverseAngle(
        float angle)
{
    float reverse_angle = angle + M_PI;
    if ( reverse_angle > M_PI )
    {
        reverse_angle -= 2*M_PI;
    }
    return reverse_angle;
}

bool Utils::isAngleWithinBounds(
        float angle,
        float max_angle,
        float min_angle)
{
    return ( min_angle < max_angle )
           ? ( angle >= min_angle && angle <= max_angle )
           : ( angle <= min_angle && angle >= max_angle );
}

PointVec2D Utils::generatePerpendicularPointsAt(
        const Pose2D& pose,
        float max_perp_dist,
        float step_size)
{
    float perpendicular_angle = Utils::calcPerpendicularAngle(pose.theta);

    Vector2D unit_vec(std::cos(perpendicular_angle), std::sin(perpendicular_angle)); 
    Point2D pt(pose.x, pose.y);
    PointVec2D pts;
    for ( float perp_dist = step_size; perp_dist < max_perp_dist; perp_dist += step_size )
    {
        Point2D offset = unit_vec * perp_dist;
        pts.push_back(pt + offset);
        pts.push_back(pt - offset);
    }
    return pts;
}

float Utils::calcAngleBetweenPoints(
        const Point2D& a,
        const Point2D& b,
        const Point2D& c)
{
    Vector2D vec_b_a = a - b;
    Vector2D vec_b_c = c - b;
    return Utils::clipAngle(std::atan2(vec_b_c.y, vec_b_c.x) -
                            std::atan2(vec_b_a.y, vec_b_a.x));
}

WindingOrder Utils::calcWindingOrder(
        const Point2D& a,
        const Point2D& b,
        const Point2D& c,
        float tolerance)
{
    float angle = calcAngleBetweenPoints(a, b, c);
    bool isCollinear = ( std::fabs(angle) <= tolerance || std::fabs(std::fabs(angle) - M_PI) <= tolerance );
    return ( isCollinear ) ? WindingOrder::COLLINEAR : 
           ( angle > 0 )   ? WindingOrder::CLOCKWISE : WindingOrder::COUNTER_CLOCKWISE;

}

void Utils::convertQuaternionToEuler(
        float qx,
        float qy,
        float qz,
        float qw,
        float& roll,
        float& pitch,
        float& yaw)
{
    /**
     * source: https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
     */
    float sinpitch = 2 * (qw * qy - qz * qx);
    if ( sinpitch >= 1.0f ) // singularity at north pole
    {
        roll = 0.0f;
        pitch = M_PI/2;
        yaw = 2 * std::atan2(qx, qw);
    }
    else if ( sinpitch <= -1.0f ) // singularity at south pole
    {
        roll = 0.0f;
        pitch = -M_PI/2;
        yaw = -2 * std::atan2(qx, qw);
    }
    else
    {
        roll = std::atan2(2 * (qx*qw + qy*qz), 1 - (2 * (qx*qx + qy*qy)));
        pitch = std::asin(sinpitch);
        yaw = std::atan2(2 * (qz*qw + qx*qy), 1 - (2 * (qy*qy + qz*qz)));
    }
}

void Utils::convertEulerToQuaternion(
        float roll,
        float pitch,
        float yaw,
        float& qx,
        float& qy,
        float& qz,
        float& qw)
{
    /**
     * source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_code
     */
    float cy = std::cos(yaw * 0.5f);
    float sy = std::sin(yaw * 0.5f);
    float cp = std::cos(pitch * 0.5f);
    float sp = std::sin(pitch * 0.5f);
    float cr = std::cos(roll * 0.5f);
    float sr = std::sin(roll * 0.5f);

    qw = (cr * cp * cy) + (sr * sp * sy);
    qx = (sr * cp * cy) - (cr * sp * sy);
    qy = (cr * sp * cy) + (sr * cp * sy);
    qz = (cr * cp * sy) - (sr * sp * cy);
}

nav_msgs::Path Utils::convertToROSPath(
        const std::vector<Pose2D>& trajectory,
        const std::string& frame)
{
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = frame;
    path_msg.poses.clear();

    for ( Pose2D pose : trajectory )
    {
        path_msg.poses.push_back(pose.asPoseStamped(frame));
    }
    return path_msg;
}

visualization_msgs::Marker Utils::convertGeometricPathToMarker(
        const Path& geometric_path,
        const std::string& frame,
        float red,
        float green,
        float blue,
        float alpha,
        float line_width)
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.header.frame_id = frame;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.scale.x = line_width;
    marker.pose.orientation.w = 1.0f;
    marker.points.reserve(geometric_path.size());
    for ( size_t i = 0; i < geometric_path.size(); i++ )
    {
        marker.points.push_back(geometric_path[i].position().asPoint());
    }
    return marker;
}

template <typename T>
visualization_msgs::Marker Utils::convertPointCloudToMarker(
        const std::vector<T>& cloud,
        const std::string& frame,
        float diameter,
        float red,
        float green,
        float blue,
        float alpha)
{
    visualization_msgs::Marker cloud_marker;
    cloud_marker.type = visualization_msgs::Marker::POINTS;
    cloud_marker.pose.orientation.w = 1.0f;
    cloud_marker.scale.x = diameter;
    cloud_marker.scale.y = diameter;
    cloud_marker.color.r = red;
    cloud_marker.color.g = green;
    cloud_marker.color.b = blue;
    cloud_marker.color.a = alpha;
    cloud_marker.header.frame_id = frame;
    cloud_marker.points.reserve(cloud.size());
    for ( const T& pt : cloud )
    {
        cloud_marker.points.push_back(pt.asPoint());
    }
    return cloud_marker;
}
template visualization_msgs::Marker Utils::convertPointCloudToMarker(
        const PointCloud2D& cloud, const std::string& frame,
        float diameter, float red, float green, float blue, float alpha);
template visualization_msgs::Marker Utils::convertPointCloudToMarker(
        const PointCloud3D& cloud, const std::string& frame,
        float diameter, float red, float green, float blue, float alpha);

visualization_msgs::Marker Utils::convertStringToMarker(
        const std::string& string_label,
        const std::string& frame,
        float red,
        float green,
        float blue,
        float alpha,
        float size)
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.header.frame_id = frame;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.scale.z = size;
    marker.pose.orientation.w = 1.0f;
    marker.text = string_label;
    return marker;
}

} // namespace geometry_common
} // namespace kelo
