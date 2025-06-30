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
#include "geometry_common/TransformMatrix2D.h"
#include "geometry_common/Pose2D.h"

namespace kelo
{
namespace geometry_common
{

Pose2D::Pose2D(float _x, float _y, float _theta)
{
    x = _x;
    y = _y;
    theta = Utils::clipAngle(_theta);
}

Pose2D::Pose2D(const geometry_msgs::msg::PoseStamped& pose)
{
    x = pose.pose.position.x;
    y = pose.pose.position.y;
    float roll, pitch;
    Utils::convertQuaternionToEuler(
            pose.pose.orientation.x, pose.pose.orientation.y,
            pose.pose.orientation.z, pose.pose.orientation.w,
            roll, pitch, theta);
}

Pose2D::Pose2D(const geometry_msgs::msg::Pose& pose)
{
    x = pose.position.x;
    y = pose.position.y;
    float roll, pitch;
    Utils::convertQuaternionToEuler(
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
            roll, pitch, theta);
}

Pose2D::Pose2D(const TransformMatrix2D& tf_mat)
{
    x = tf_mat.x();
    y = tf_mat.y();
    theta = tf_mat.theta();
}

Pose2D::Pose2D(const geometry_msgs::msg::TransformStamped& ts)
{
    TransformMatrix2D tf_mat(ts);
    x = tf_mat.x();
    y = tf_mat.y();
    theta = tf_mat.theta();
}

void Pose2D::updatePosition(const Point2D& pt)
{
    x = pt.x;
    y = pt.y;
}

geometry_msgs::msg::PoseStamped Pose2D::asPoseStamped(const std::string& frame) const
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame;
    pose.pose = asPose();
    return pose;
}

geometry_msgs::msg::Pose Pose2D::asPose() const
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.0f;
    pose.orientation.x = 0.0f;
    pose.orientation.y = 0.0f;
    float qx, qy, qz, qw;
    Utils::convertEulerToQuaternion(0.0f, 0.0f, theta, qx, qy, qz, qw);
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;
    return pose;
}

TransformMatrix2D Pose2D::asMat() const
{
    return TransformMatrix2D(*this);
}

visualization_msgs::msg::Marker Pose2D::asMarker(const std::string& frame,
        float red, float green, float blue, float alpha,
        float size_x, float size_y, float size_z) const
{
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    // marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.scale.x = size_x;
    marker.scale.y = size_y;
    marker.scale.z = size_z;
    marker.pose = asPose();
    return marker;
}

visualization_msgs::msg::InteractiveMarker Pose2D::asInteractiveMarker(
        const std::string& name, const std::string& frame,
        float red, float green, float blue, float alpha,
        float size_x, float size_y, float size_z) const
{
    visualization_msgs::msg::InteractiveMarker interactive_marker;
    interactive_marker.header.frame_id = frame;
    interactive_marker.name = name;
    interactive_marker.pose = asPose();

    visualization_msgs::msg::Marker arrow_marker = asMarker(
            frame, red, green, blue, alpha, size_x, size_y, size_z);

    visualization_msgs::msg::InteractiveMarkerControl arrow_control;
    arrow_control.always_visible = true;
    arrow_control.markers.push_back(arrow_marker);
    arrow_control.name = "move_x_y";
    arrow_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    arrow_control.orientation.w = 1.0f;
    arrow_control.orientation.y = 1.0f;
    interactive_marker.controls.push_back(arrow_control);

    visualization_msgs::msg::InteractiveMarkerControl rotate_control;
    rotate_control.name = "rotate_z";
    rotate_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    rotate_control.orientation.y = 1.0f;
    rotate_control.orientation.w = 1.0f;
    interactive_marker.controls.push_back(rotate_control);

    return interactive_marker;
}

std::string Pose2D::asString() const
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << "<x: " << x
        << ", y: " << y
        << ", theta: " << theta
        << ">";
    return ss.str();
}

Pose2D& Pose2D::operator = (const Pose2D& other)
{
    x = other.x;
    y = other.y;
    theta = other.theta;
    return *this;
}

Pose2D Pose2D::operator - (const Pose2D& other) const
{
    Pose2D diff;
    diff.x = x - other.x;
    diff.y = y - other.y;
    diff.theta = Utils::calcShortestAngle(theta, other.theta);
    return diff;
}

bool Pose2D::operator == (const Pose2D& other) const
{
    return ( distTo(other) < 1e-3f &&
             Utils::calcShortestAngle(theta, other.theta) < 1e-2f );
}

bool Pose2D::operator != (const Pose2D& other) const
{
    return !((*this) == other);
}

} // namespace geometry_common
} // namespace kelo
