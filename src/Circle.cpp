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

#include "geometry_common/Polygon2D.h"
#include "geometry_common/Circle.h"

namespace kelo
{
namespace geometry_common
{

Point2D Circle::center() const
{
    return Point2D(x, y);
}

void Circle::setCenter(const Point2D& center)
{
    x = center.x;
    y = center.y;
}

bool Circle::fromPoints(const Point2D& p1, const Point2D& p2,
                        const Point2D& p3, Circle& circle)
{
    Vector2D u(p1.y - p2.y, p2.x - p1.x);
    Vector2D v(p2.y - p3.y, p3.x - p2.x);
    float vu = v.scalarCrossProduct(u);
    if ( std::fabs(vu) < 1e-6f )
    {
        return false; // points are collinear, so no unique solution
    }
    Point2D p1_p2_mid = (p1 + p2) * 0.5f;
    Point2D p2_p3_mid = (p2 + p3) * 0.5f;
    Vector2D mid_diff = p1_p2_mid - p2_p3_mid;
    float g = mid_diff.scalarCrossProduct(u) / vu;
    circle.setCenter(p2_p3_mid + (v * g));
    circle.r = circle.distTo(p1);
    return true;
}

visualization_msgs::msg::Marker Circle::asMarker(const std::string& frame,
        float red, float green, float blue, float alpha) const
{
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.header.frame_id = frame;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.scale.x = 2*r;
    marker.scale.y = 2*r;
    marker.scale.z = 2*r;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0f;
    marker.pose.orientation.w = 1.0f;
    return marker;
}

Polygon2D Circle::asPolygon2D(size_t num_of_segments) const
{
    Polygon2D polygon;
    float delta_angle = (2 * M_PI) / num_of_segments;
    float angle = -M_PI;
    const Point2D center_pt = center();
    polygon.vertices.reserve(num_of_segments);
    for ( size_t i = 0; i < num_of_segments; i++, angle += delta_angle )
    {
        polygon.vertices.push_back(center_pt + Point2D::initFromRadialCoord(r, angle));
    }
    return polygon;
}

Circle& Circle::operator = (const Circle& other)
{
    x = other.x;
    y = other.y;
    r = other.r;
    return *this;
}

bool Circle::operator == (const Circle& other) const
{
    return ( distTo(other) < 1e-3f && std::fabs(r - other.r) < 1e-3f );
}

bool Circle::operator != (const Circle& other) const
{
    return !((*this) == other);
}

std::ostream& operator << (std::ostream& out, const Circle& circle)
{
    out <<  "<x: " << circle.x
        << ", y: " << circle.y
        << ", r: " << circle.r
        << ">";
    return out;
}

} // namespace geometry_common
} // namespace kelo
