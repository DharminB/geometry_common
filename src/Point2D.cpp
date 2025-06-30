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
#include "geometry_common/Point2D.h"

namespace kelo
{
namespace geometry_common
{

Point2D Point2D::initFromRadialCoord(float radius, float angle)
{
    return Point2D(radius * std::cos(angle), radius * std::sin(angle));
}

geometry_msgs::msg::Point Point2D::asPoint() const
{
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = 0.0f;
    return point;
}

geometry_msgs::msg::Point32 Point2D::asPoint32() const
{
    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = y;
    point.z = 0.0f;
    return point;
}

geometry_msgs::msg::PointStamped Point2D::asPointStamped(const std::string& frame) const
{
    geometry_msgs::msg::PointStamped point;
    point.header.frame_id = frame;
    point.point = asPoint();
    return point;
}

void Point2D::normalise()
{
    float mag = magnitude();
    if ( mag > 0 )
    {
        x /= mag;
        y /= mag;
    }
}

Point2D Point2D::asNormalised() const
{
    Point2D normalised_pt(*this);
    normalised_pt.normalise();
    return normalised_pt;
}

float Point2D::scalarCrossProduct(const Point2D& point) const
{
    return (x * point.y) - (y * point.x);
}

float Point2D::dotProduct(const Point2D& point) const
{
    return (x * point.x) + (y * point.y);
}

float Point2D::angle() const
{
    return std::atan2(y, x);
}

visualization_msgs::msg::Marker Point2D::asMarker(const std::string& frame,
        float red, float green, float blue, float alpha, float diameter, 
        float z) const
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

visualization_msgs::msg::InteractiveMarker Point2D::asInteractiveMarker(
        const std::string& name, const std::string& frame,
        float red, float green, float blue, float alpha,
        float diameter, float z) const
{
    visualization_msgs::msg::InteractiveMarker interactive_marker;
    interactive_marker.header.frame_id = frame;
    interactive_marker.name = name;
    interactive_marker.pose.position = asPoint();
    interactive_marker.pose.orientation.w = 1.0f;

    visualization_msgs::msg::Marker sphere_marker = asMarker(
            frame, red, green, blue, alpha, diameter, z);

    visualization_msgs::msg::InteractiveMarkerControl sphere_control;
    sphere_control.always_visible = true;
    sphere_control.markers.push_back(sphere_marker);
    sphere_control.name = "move_x_y";
    sphere_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    float qx, qy, qz, qw;
    Utils::convertEulerToQuaternion(0.0f, M_PI/2, 0.0f, qx, qy, qz, qw);
    sphere_control.orientation.x = qx;
    sphere_control.orientation.y = qy;
    sphere_control.orientation.z = qz;
    sphere_control.orientation.w = qw;
    interactive_marker.controls.push_back(sphere_control);

    return interactive_marker;
}

Point2D& Point2D::operator = (const Point2D& other)
{
    x = other.x;
    y = other.y;
    return *this;
}

Point2D Point2D::operator - (const Point2D& other) const
{
    Point2D diff;
    diff.x = x - other.x;
    diff.y = y - other.y;
    return diff;
}

Point2D Point2D::operator + (const Point2D& other) const
{
    Point2D sum;
    sum.x = x + other.x;
    sum.y = y + other.y;
    return sum;
}

Point2D Point2D::operator * (float scalar) const
{
    Point2D scaled;
    scaled.x = x * scalar;
    scaled.y = y * scalar;
    return scaled;
}

Point2D Point2D::operator / (float scalar) const
{
    if ( std::fabs(scalar) < 1e-9f ) // to fix divide by zero issue
    {
        scalar = 1e-9f;
    }
    return (*this) * (1.0f/scalar);
}

bool Point2D::operator == (const Point2D& other) const
{
    return ( distTo(other) < 1e-3f );
}

bool Point2D::operator != (const Point2D& other) const
{
    return !((*this) == other);
}

std::ostream& operator << (std::ostream& out, const Point2D& point)
{
    out <<  "<x: " << point.x
        << ", y: " << point.y
        << ">";
    return out;
}

} // namespace geometry_common
} // namespace kelo
