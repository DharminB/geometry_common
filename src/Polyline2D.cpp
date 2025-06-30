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

#include "geometry_common/LineSegment2D.h"
#include "geometry_common/Point3D.h"
#include "geometry_common/Polyline2D.h"

namespace kelo
{
namespace geometry_common
{

float Polyline2D::length() const
{
    float len = 0.0f;
    for ( unsigned int start = 0, end = start + 1; end < vertices.size(); start = end++ )
    {
        len += vertices[start].distTo(vertices[end]);
    }
    return len;
}

bool Polyline2D::intersects(const LineSegment2D& line_segment) const
{
    // Conditions also ensures that there are atleast 2 vertices
    for ( size_t start = 0, end = start + 1; end < vertices.size(); start = end++ )
    {
        if ( LineSegment2D(vertices[start], vertices[end]).intersects(line_segment) )
        {
            return true;
        }
    }
    return false;
}

bool Polyline2D::intersects(const Polyline2D& polyline) const
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

bool Polyline2D::calcClosestIntersectionPointWith(
        const LineSegment2D& line_segment,
        Point2D& intersection_pt) const
{
    bool intersects = false;
    float minDist = std::numeric_limits<float>::max();
    for ( unsigned int start = 0, end = start + 1; end < vertices.size(); start = end++ )
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

bool Polyline2D::calcClosestIntersectionPoseWith(
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

std::vector<LineSegment2D> Polyline2D::split(float max_segment_length) const
{
    std::vector<LineSegment2D> segments;
    for ( unsigned int start = 0, end = start + 1; end < vertices.size();
         start = end++ )
    {
        LineSegment2D edge(vertices[start], vertices[end]);
        Point2D unit_vector = edge.unitVector();
        if ( max_segment_length > 0 )
        {
            while ( edge.length() > max_segment_length )
            {
                Point2D split_point = edge.start + (unit_vector * max_segment_length);
                segments.push_back(LineSegment2D(edge.start, split_point));
                edge.start = split_point;
            }
        }
        segments.push_back(edge);
    }
    return segments;
}

void Polyline2D::reverse()
{
    std::reverse(vertices.begin(), vertices.end());
}

visualization_msgs::msg::Marker Polyline2D::asMarker(const std::string& frame,
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
            if ( !use_line_strip && i > 0 && i+1 < vertices.size() )
            {
                marker.points.push_back(vertices[i].asPoint());
            }
        }
    }
    return marker;
}

Polyline2D& Polyline2D::operator = (const Polyline2D& other)
{
    vertices = other.vertices;
    return *this;
}

Point2D& Polyline2D::operator [] (unsigned int index)
{
    return vertices[index];
}

const Point2D& Polyline2D::operator [] (unsigned int index) const
{
    return vertices[index];
}

bool Polyline2D::operator == (const Polyline2D& other) const
{
    if ( vertices.size() != other.size() )
    {
        return false;
    }

    for ( size_t i = 0; i < vertices.size(); i++ )
    {
        if ( !(vertices[i] == other[i]) )
        {
            return false;
        }
    }
    return true;
}

bool Polyline2D::operator != (const Polyline2D& other) const
{
    return !((*this) == other);
}

std::ostream& operator<<(std::ostream& out, const Polyline2D& polyline)
{
    out << "<Polyline vertices: [";
    for ( size_t i = 0; i < polyline.vertices.size(); i++ )
    {
        if ( i > 0 )
        {
            out << ", ";
        }
        out << polyline.vertices[i];
    }
    out << "]>";
    return out;
}

} // namespace geometry_common
} // namespace kelo
