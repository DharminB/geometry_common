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
#include "geometry_common/Point3D.h"
#include "geometry_common/LineSegment2D.h"

namespace kelo
{
namespace geometry_common
{

LineSegment2D::~LineSegment2D()
{
}

float LineSegment2D::angle() const
{
    Vector2D diff = end - start;
    return std::atan2(diff.y, diff.x);
}

float LineSegment2D::length() const
{
    return start.distTo(end);
}

float LineSegment2D::slope() const
{
    Vector2D diff = end - start;
    if ( std::fabs(diff.x) < 1e-6f )
    {
        diff.x = 1e-6f;
    }
    return diff.y/diff.x;
}

float LineSegment2D::constant() const
{
    float m = slope();
    return start.y - (m * start.x);
}

Point2D LineSegment2D::center() const
{
    return (start + end) * 0.5f;
}

Point2D LineSegment2D::unitVector() const
{
    return (end - start) / static_cast<float>(length());
}

bool LineSegment2D::intersects(const LineSegment2D& line_segment) const
{
    Point2D intersection_pt;
    return calcIntersectionPointWith(line_segment, intersection_pt);
}

bool LineSegment2D::calcIntersectionPointWith(
        const LineSegment2D& line_segment,
        Point2D& intersection_point,
        bool is_outside_allowed) const
{
    /**
     * source: https://stackoverflow.com/a/565282/10460994
     */
    Vector2D vec1 = end - start;
    Vector2D vec2 = line_segment.end - line_segment.start;
    Vector2D vec3 = line_segment.start - start;
    const float vec1_cross_vec2 = vec1.scalarCrossProduct(vec2);
    const float vec3_cross_vec1 = vec3.scalarCrossProduct(vec1);
    const float vec3_cross_vec2 = vec3.scalarCrossProduct(vec2);

    if ( std::fabs(vec1_cross_vec2) < 1e-10f &&
         std::fabs(vec3_cross_vec1) < 1e-10f ) // the two lines are collinear
    {
        const float t0 = vec3.dotProduct(vec1) / vec1.dotProduct(vec1);
        const float t1 = t0 + (vec2.dotProduct(vec1) / vec1.dotProduct(vec1));
        const bool are_lines_opposite = ( vec2.dotProduct(vec1) < 0.0f );
        // If the interval between t0 and t1 intersects the interval [0, 1] then
        // the line segments are collinear and overlapping; otherwise they are
        // collinear and disjoint. If vec2 and vec1 point in opposite
        // directions, then vec2 . vec1 < 0 and so the interval to be checked is
        // [t1, t0] rather than [t0, t1].
        if ( ( !are_lines_opposite && (1.0f < t0 || t1 < 0.0f) ) ||
             (  are_lines_opposite && (1.0f < t1 || t0 < 0.0f) ) )
        {
            return false;
        }
        // Ideally the intersection is a smaller line segment but here the start
        // of that line segment is chosen
        intersection_point = start + (vec1 * std::max(0.0f, std::min(t0, t1)));
        return true;
    }

    if ( std::fabs(vec1_cross_vec2) < 1e-10f &&
         std::fabs(vec3_cross_vec1) > 1e-10f ) // the two lines are parallel and non-intersecting
    {
        return false;
    }

    const float t = vec3_cross_vec2/vec1_cross_vec2;
    const float u = vec3_cross_vec1/vec1_cross_vec2;

    if ( std::fabs(vec1_cross_vec2) > 1e-10f ) // The two line segments are not parallel
    {
        if ( !is_outside_allowed &&
             (t < 0.0f || t > 1.0f ||
              u < 0.0f || u > 1.0f) )
        {
            return false; // The two line segments intersect outside of bounds
        }
    }

    intersection_point = start + (vec1 * t);
    return true;
}

Point2D LineSegment2D::closestPointTo(const Point2D& point) const
{
    return Utils::calcProjectedPointOnLine(start, end, point, true);
}

float LineSegment2D::minDistTo(const Point2D& point) const
{
    return point.distTo(closestPointTo(point));
}

float LineSegment2D::squaredMinDistTo(const Point2D& p) const
{
    return Utils::calcSquaredDistToLine(start, end, p, true);
}

bool LineSegment2D::containsPoint(
        const Point2D& point,
        float dist_threshold) const
{
    return ( minDistTo(point) < dist_threshold );
}

visualization_msgs::msg::Marker LineSegment2D::asMarker(const std::string& frame,
        float red, float green, float blue, float alpha, float line_width) const
{
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.header.frame_id = frame;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.scale.x = line_width;
    marker.pose.orientation.w = 1.0f;
    marker.points.push_back(start.asPoint());
    marker.points.push_back(end.asPoint());
    return marker;
}

LineSegment2D& LineSegment2D::operator = (const LineSegment2D& other)
{
    start = other.start;
    end = other.end;
    return *this;
}

bool LineSegment2D::operator == (const LineSegment2D& other) const
{
    return ( start == other.start && end == other.end );
}

bool LineSegment2D::operator != (const LineSegment2D& other) const
{
    return !((*this) == other);
}

std::ostream& operator << (std::ostream& out, const LineSegment2D& line_segment)
{
    out << "<start: " << line_segment.start
        << ", end: " << line_segment.end << ">";
    return out;
}

} // namespace geometry_common
} // namespace kelo
