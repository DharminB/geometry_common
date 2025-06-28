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

#include <cmath>

#include "geometry_common/Utils.h"
#include "geometry_common/Polygon2D.h"

namespace kelo
{
namespace geometry_common
{

float Polygon2D::length() const
{
    size_t nVertices = vertices.size();
    if ( nVertices < 2 )
        return 0.0f;

    return Polyline2D::length() + vertices[0].distTo(vertices[nVertices - 1]);
}

bool Polygon2D::intersects(const LineSegment2D& line_segment) const
{
    for ( size_t start = vertices.size() - 1, end = 0; end < vertices.size(); start = end++ )
    {
        if ( LineSegment2D(vertices[start], vertices[end]).intersects(line_segment) )
        {
            return true;
        }
    }
    return false;
}

bool Polygon2D::intersects(const Polyline2D& polyline) const
{
    for ( size_t start = 0, end = start + 1; end < polyline.size(); start = end++ )
    {
        if ( intersects(LineSegment2D(polyline[start], polyline[end])) )
        {
            return true;
        }
    }
    return false;
}

bool Polygon2D::calcClosestIntersectionPointWith(
        const LineSegment2D& line_segment,
        Point2D& intersection_pt) const
{
    bool intersects = false;
    float minDist = std::numeric_limits<float>::max();
    for ( unsigned int start = vertices.size() - 1, end = 0; end < vertices.size(); start = end++ )
    {
        Point2D pt;
        if ( line_segment.calcIntersectionPointWith(
                    LineSegment2D(vertices[start], vertices[end]), pt) )
        {
            float dist = line_segment.start.distTo(pt);
            if (dist < minDist)
            {
                minDist = dist;
                intersection_pt = pt;
                intersects = true;
            }
        }
    }
    return intersects;
}

bool Polygon2D::calcClosestIntersectionPoseWith(
        const Polyline2D& polyline,
        Pose2D& intersection_pose,
        unsigned int& segment_id) const
{
    for ( unsigned int start = 0, end = start + 1; end < polyline.size();
         start = end++ )
    {
        LineSegment2D segment(polyline[start], polyline[end]);
        Point2D intersection_pt;
        if ( calcClosestIntersectionPointWith(segment, intersection_pt) )
        {
            intersection_pose = Pose2D(intersection_pt, segment.angle());
            segment_id = start;
            return true;
        }
    }
    return false;
}

bool Polygon2D::containsPoint(const Point2D& point) const
{
    /* 
     * Source: https://stackoverflow.com/a/2922778/10460994
     */
    size_t i, j, counter = 0;
    for ( i = 0, j = vertices.size() - 1; i < vertices.size(); j = i++ )
    {
        const Point2D& currVert = vertices[i];
        const Point2D& prevVert = vertices[j];
        if ( ((currVert.y > point.y) != (prevVert.y > point.y)) &&
             (point.x < (prevVert.x - currVert.x) * (point.y - currVert.y) / (prevVert.y - currVert.y) + currVert.x) )
        {
            counter++;
        }
    }
    return ( counter % 2 == 1 );
}

bool Polygon2D::containsAnyPoint(const PointVec2D& points) const
{
    for ( const Point2D& pt: points )
    {
        if ( containsPoint(pt) )
        {
            return true;
        }
    }
    return false; 
}

Point2D Polygon2D::meanPoint() const
{
    Point2D mean;
    if (!vertices.empty())
    {
        for (const auto& vertex: vertices)
        {
            mean = mean + vertex;
        }
        mean = mean / static_cast<float>(vertices.size());
    }
    return mean;
}

float Polygon2D::area() const
{
    float area = 0.0f;
    size_t i, j;
    for ( i = 0; i < vertices.size(); i++ )
    {
        j = (i+1) % vertices.size();
        area += (vertices[i].x * vertices[j].y) - (vertices[i].y * vertices[j].x);
    }
    return area/2;
}

bool Polygon2D::isConvex() const
{
    if ( vertices.size() <= 2 )
    {
        return true;
    }

    Point2D meanPt = meanPoint();
    if ( !containsPoint(meanPt) )
    {
        return false;
    }

    for ( size_t i = 0; i < vertices.size(); i++ )
    {
        Point2D p1(vertices[i]);
        Point2D p2(vertices[(i+2)%vertices.size()]);
        meanPt = (p1 + p2) * 0.5f;
        if ( !containsPoint(meanPt) )
        {
            return false;
        }
    }
    return true;
}

bool Polygon2D::isApproximatelyConvex(float tolerance) const
{
    size_t n_vertices = vertices.size();
    if ( n_vertices <= 2 )
    {
        return true;
    }

    Point2D meanPt = meanPoint();
    if ( !containsPoint(meanPt) )
    {
        return false;
    }

    for ( size_t i = 0; i < n_vertices; i++ )
    {
        const Point2D& p1 = vertices[i];
        const Point2D& p2 = vertices[(i+1)%n_vertices];
        const Point2D& p3 = vertices[(i+2)%n_vertices];

        // Ignore repeated points
        if (p2 == p3)
            continue;

        meanPt = (p1 + p3) * 0.5f;
        // If the mean point is not contained inside the polygon, check if the
        // three consecutive points are collinear within the tolerance. If they
        // not collinear, the polygon is not convex
        if ( !containsPoint(meanPt) &&
             Utils::calcWindingOrder(p1, p2, p3, tolerance) != WindingOrder::COLLINEAR )
        {
            return false;
        }
    }
    return true;
}

Polygon2D Polygon2D::calcConvexHullOfPolygons(
        const Polygon2D& polygon_a,
        const Polygon2D& polygon_b)
{
    /**
     * source: https://en.wikipedia.org/wiki/Graham_scan#Pseudocode
     */

    /* aggregate all points */
    PointVec2D pts;
    pts.reserve(pts.size() + polygon_a.vertices.size() + polygon_b.vertices.size());
    pts.insert(pts.end(), polygon_a.vertices.begin(), polygon_a.vertices.end());
    pts.insert(pts.end(), polygon_b.vertices.begin(), polygon_b.vertices.end());
    if ( pts.size() < 3 )
    {
        return pts;
    }

    /* find the lowest left most point */
    Point2D lower_left_pt(pts[0]);
    for ( size_t i = 0; i < pts.size(); i++ )
    {
        if ( pts[i].y < lower_left_pt.y )
        {
            lower_left_pt = pts[i];
        }
        else if ( pts[i].y == lower_left_pt.y && pts[i].x < lower_left_pt.x )
        {
            lower_left_pt = pts[i];
        }
    }

    /* sort points in increasing order of angle they and lower_left_pt makes with X axis */
    std::sort(pts.begin(), pts.end(),
              [&lower_left_pt](const Point2D& a, const Point2D& b)
              {
                  return std::atan2(a.y - lower_left_pt.y, a.x - lower_left_pt.x)
                       < std::atan2(b.y - lower_left_pt.y, b.x - lower_left_pt.x);
              });

    /* walk along pts and remove points that form non counter clockwise turn */
    PointVec2D convex_hull;
    for ( Point2D& p : pts )
    {
        while ( convex_hull.size() > 1 )
        {
            PointVec2D::const_iterator it = convex_hull.end();
            float angle = Utils::calcAngleBetweenPoints(p, *(it-1), *(it-2));
            if ( angle > 0 ) // counter clockwise turn is allowed
            {
                break;
            }
            convex_hull.pop_back();
        }
        convex_hull.push_back(p);
    }
    return Polygon2D(convex_hull);
}

Polygon2D Polygon2D::calcInflatedPolygon(float inflation_dist) const
{
    Polygon2D inflated_polygon(*this);
    size_t N = vertices.size();
    if ( N < 3 )
    {
        return inflated_polygon;
    }

    // calculate shortest side length
    float shortest_side_len = 1e6f;
    for ( size_t i = 0; i < N; i++ )
    {
        float side_len = vertices[i].distTo(vertices[(i+1)%N]);
        if ( side_len < shortest_side_len )
        {
            shortest_side_len = side_len;
        }
    }
    float offset_dist = shortest_side_len/10; // just a rule of thumb

    // inflate each point in the polygon based on the angle with prev and next pt
    for ( size_t i = 0; i < N; i++ )
    {
        const Point2D& a = vertices[(i+N-1) % N]; // prev pt
        const Point2D& b = vertices[i]; // current pt
        const Point2D& c = vertices[(i+1) % N]; // next pt

        const Vector2D vec_b_a = (a - b).asNormalised();
        const Vector2D vec_b_c = (c - b).asNormalised();

        // calculate inflation dist based on the angle formed between a, b and c
        float theta = std::fabs(Utils::calcAngleBetweenPoints(a, b, c));
        float rhombus_side_length = inflation_dist / std::sin(theta);
        float diag_1_length = 2 * rhombus_side_length * std::cos(theta/2);
        float diag_2_length = 2 * rhombus_side_length * std::sin(theta/2);
        float bigger_diag_length = ( std::fabs(diag_1_length) > std::fabs(diag_2_length) )
                                   ? diag_1_length : diag_2_length;
        float smaller_diag_length = ( std::fabs(diag_1_length) > std::fabs(diag_2_length) )
                                    ? diag_2_length : diag_1_length;
        float scaled_inflation_dist = ( std::fabs(theta) < M_PI/2 )
                                      ? bigger_diag_length : smaller_diag_length;

        // calculate normalised vec along which to move current vertex
        Point2D intermediate_a = b + (vec_b_a * offset_dist);
        Point2D intermediate_c = b + (vec_b_c * offset_dist);
        Point2D mid_intermediate_pt = (intermediate_a + intermediate_c) * 0.5f;
        Vector2D diff = (mid_intermediate_pt - b).asNormalised();

        // calculate sign of inflation dist based on direction of normalised vec
        Point2D test_pt = b + (diff * offset_dist);
        int sign = ( containsPoint(test_pt) ) ? -1 : 1;

        Point2D final_pt = b + (diff * (sign * scaled_inflation_dist));
        inflated_polygon.vertices[i] = final_pt;
    }
    return inflated_polygon;
}

visualization_msgs::msg::Marker Polygon2D::asMarker(const std::string& frame,
        float red, float green, float blue, float alpha, float line_width,
        bool use_line_strip) const
{
    visualization_msgs::msg::Marker marker;
    marker.type = ( use_line_strip )
                  ? visualization_msgs::msg::Marker::LINE_STRIP
                  : visualization_msgs::msg::Marker::LINE_LIST;
    marker.header.frame_id = frame;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.scale.x = line_width;
    marker.pose.orientation.w = 1.0f;
    if (!vertices.empty())
    {
        if ( use_line_strip )
        {
            marker.points.reserve(vertices.size());
        }
        else
        {
            marker.points.reserve(vertices.size()*2);
        }
        for ( size_t i = 0; i < vertices.size(); i++ )
        {
            marker.points.push_back(vertices[i].asPoint());
            if ( !use_line_strip && i > 0 )
            {
                marker.points.push_back(vertices[i].asPoint());
            }
        }
        // Repeat first point again to close the polygon loop
        marker.points.push_back(vertices[0].asPoint());
    }
    return marker;
}

geometry_msgs::msg::PolygonStamped Polygon2D::asPolygonStamped(
        const std::string& frame) const
{
    geometry_msgs::msg::PolygonStamped polygon_msg;
    polygon_msg.header.frame_id = frame;
    polygon_msg.polygon.points.reserve(vertices.size());
    for ( const Point2D& pt : vertices )
    {
        polygon_msg.polygon.points.push_back(pt.asPoint32());
    }
    return polygon_msg;
}

Polygon2D& Polygon2D::operator = (const Polygon2D& other)
{
    vertices = other.vertices;
    return *this;
}

std::ostream& operator<<(std::ostream& out, const Polygon2D& polygon)
{
    out << "<Polygon vertices: [";
    for ( size_t i = 0; i < polygon.vertices.size(); i++ )
    {
        if ( i > 0 )
        {
            out << ", ";
        }
        out << polygon.vertices[i];
    }
    out << "]>";
    return out;
}

} // namespace geometry_common
} // namespace kelo
