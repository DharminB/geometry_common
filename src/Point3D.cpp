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

#include "geometry_common/Utils.h"
#include "geometry_common/Point3D.h"

namespace kelo
{
namespace geometry_common
{

geometry_msgs::msg::Point Point3D::asPoint() const
{
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

geometry_msgs::msg::Point32 Point3D::asPoint32() const
{
    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

geometry_msgs::msg::PointStamped Point3D::asPointStamped(const std::string& frame) const
{
    geometry_msgs::msg::PointStamped point;
    // point.header.stamp = ros::Time::now();
    point.header.frame_id = frame;
    point.point = asPoint();
    return point;
}

void Point3D::normalise()
{
    float mag = magnitude();
    if ( mag > 0 )
    {
        x /= mag;
        y /= mag;
        z /= mag;
    }
}

Point3D Point3D::asNormalised() const
{
    Point3D normalised_pt(*this);
    normalised_pt.normalise();
    return normalised_pt;
}

float Point3D::dotProduct(const Point3D& point) const
{
    return (x * point.x) + (y * point.y) + (z * point.z);
}

visualization_msgs::msg::Marker Point3D::asMarker(const std::string& frame,
        float red, float green, float blue, float alpha, float diameter) const
{
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.header.frame_id = frame;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.scale.x = diameter;
    marker.scale.y = diameter;
    marker.scale.z = diameter;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0f;
    return marker;
}

Point3D& Point3D::operator = (const Point3D& other)
{
    x = other.x;
    y = other.y;
    z = other.z;
    return *this;
}

Point3D Point3D::operator - (const Point3D& other) const
{
    Point3D diff;
    diff.x = x - other.x;
    diff.y = y - other.y;
    diff.z = z - other.z;
    return diff;
}

Point3D Point3D::operator + (const Point3D& other) const
{
    Point3D sum;
    sum.x = x + other.x;
    sum.y = y + other.y;
    sum.z = z + other.z;
    return sum;
}

Point3D Point3D::operator * (float scalar) const
{
    Point3D scaled;
    scaled.x = x * scalar;
    scaled.y = y * scalar;
    scaled.z = z * scalar;
    return scaled;
}

Point3D Point3D::operator / (float scalar) const
{
    if ( std::fabs(scalar) < 1e-9f ) // to fix divide by zero issue
    {
        scalar = 1e-9f;
    }
    return (*this) * (1.0f/scalar);
}

bool Point3D::operator == (const Point3D& other) const
{
    return ( distTo(other) < 1e-3f );
}

bool Point3D::operator != (const Point3D& other) const
{
    return !((*this) == other);
}

std::ostream& operator << (std::ostream& out, const Point3D& point)
{
    out <<  "<x: " << point.x
        << ", y: " << point.y
        << ", z: " << point.z
        << ">";
    return out;
}

} // namespace geometry_common
} // namespace kelo
