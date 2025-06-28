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
#include "geometry_common/Pose2D.h"
#include "geometry_common/Point2D.h"
#include "geometry_common/Circle.h"
#include "geometry_common/LineSegment2D.h"
#include "geometry_common/Polyline2D.h"
#include "geometry_common/Polygon2D.h"
#include "geometry_common/TransformMatrix2D.h"

namespace kelo
{
namespace geometry_common
{

TransformMatrix2D::TransformMatrix2D(float x, float y, float theta)
{
    update(x, y, theta);
}

TransformMatrix2D::TransformMatrix2D(
        float x, float y, float qx, float qy, float qz, float qw)
{
    update(x, y, qx, qy, qz, qw);
}

TransformMatrix2D::TransformMatrix2D(
        const geometry_msgs::msg::TransformStamped& ts)
{
    update(ts);
}

TransformMatrix2D::TransformMatrix2D(const Pose2D& pose)
{
    update(pose);
}
TransformMatrix2D::TransformMatrix2D(const XYTheta& x_y_theta)
{
    update(x_y_theta);
}

TransformMatrix2D::TransformMatrix2D(const TransformMatrix2D& tf_mat)
{
    update(tf_mat);
}

void TransformMatrix2D::update(float x, float y, float theta)
{
    updateX(x);
    updateY(y);
    updateTheta(theta);
}

void TransformMatrix2D::update(
        float x, float y, float qx, float qy, float qz, float qw)
{
    updateX(x);
    updateY(y);
    updateQuaternion(qx, qy, qz, qw);
}

void TransformMatrix2D::update(
        const geometry_msgs::msg::TransformStamped& ts)
{
    update(
        ts.transform.translation.x,
        ts.transform.translation.y,
        ts.transform.rotation.x,
        ts.transform.rotation.y,
        ts.transform.rotation.z,
        ts.transform.rotation.w);
}

void TransformMatrix2D::update(const Pose2D& pose)
{
    update(pose.x, pose.y, pose.theta);
}

void TransformMatrix2D::update(const XYTheta& x_y_theta)
{
    update(x_y_theta.x, x_y_theta.y, x_y_theta.theta);
}

void TransformMatrix2D::update(const TransformMatrix2D& tf_mat)
{
    for ( size_t i = 0; i < mat_.size(); i++ )
    {
        mat_[i] = tf_mat[i];
    }
}

void TransformMatrix2D::updateX(float x)
{
    mat_[2] = x;
}

void TransformMatrix2D::updateY(float y)
{
    mat_[5] = y;
}

void TransformMatrix2D::updateTheta(float theta)
{
    mat_[0] = std::cos(theta);
    mat_[1] = -std::sin(theta);
    mat_[3] = std::sin(theta);
    mat_[4] = std::cos(theta);
}

void TransformMatrix2D::updateQuaternion(float qx, float qy, float qz, float qw)
{
    float roll, pitch, yaw;
    Utils::convertQuaternionToEuler(qx, qy, qz, qw, roll, pitch, yaw);
    updateTheta(yaw);
}

TransformMatrix2D TransformMatrix2D::calcInverse() const
{
    TransformMatrix2D inv_tf_mat(*this);
    inv_tf_mat.invert();
    return inv_tf_mat;
}

void TransformMatrix2D::invert()
{
    // taking transpose of rotation matrix part since
    // inverse(M) == transpose(M) : where M is a orthonormal rotation matrix
    float temp = mat_[1];
    mat_[1] = mat_[3];
    mat_[3] = temp;

    // - inverse(M) * T : where M is as above and T is translation vector
    float x = mat_[2];
    float y = mat_[5];
    mat_[2] = -((mat_[0] * x) + (mat_[1] * y));
    mat_[5] = -((mat_[3] * x) + (mat_[4] * y));
}

float TransformMatrix2D::x() const
{
    return mat_[2];
}

float TransformMatrix2D::y() const
{
    return mat_[5];
}

float TransformMatrix2D::theta() const
{
    return std::atan2(mat_[3], mat_[0]);
}

std::array<float, 4> TransformMatrix2D::quaternion() const
{
    std::array<float, 4> q;
    Utils::convertEulerToQuaternion(0.0f, 0.0f, theta(), q[0], q[1], q[2], q[3]);
    return q;
}

std::array<float, 4> TransformMatrix2D::rotationMatrix() const
{
    std::array<float, 4> rot_mat;
    rot_mat[0] = mat_[0];
    rot_mat[1] = mat_[1];
    rot_mat[2] = mat_[3];
    rot_mat[3] = mat_[4];
    return rot_mat;
}

Vector2D TransformMatrix2D::translationVector() const
{
    return Vector2D(mat_[2], mat_[5]);
}

Pose2D TransformMatrix2D::asPose2D() const
{
    return Pose2D(mat_[2], mat_[5], theta());
}

void TransformMatrix2D::transform(Point2D& point) const
{
    float temp_x = (mat_[0] * point.x) + (mat_[1] * point.y) + mat_[2];
    float temp_y = (mat_[3] * point.x) + (mat_[4] * point.y) + mat_[5];
    point.x = temp_x;
    point.y = temp_y;
}

void TransformMatrix2D::transform(Circle& circle) const
{
    float temp_x = (mat_[0] * circle.x) + (mat_[1] * circle.y) + mat_[2];
    float temp_y = (mat_[3] * circle.x) + (mat_[4] * circle.y) + mat_[5];
    circle.x = temp_x;
    circle.y = temp_y;
}

void TransformMatrix2D::transform(Pose2D& pose) const
{
    TransformMatrix2D transformed_mat = (*this) * pose.asMat();
    pose.x = transformed_mat[2];
    pose.y = transformed_mat[5];
    pose.theta = std::atan2(transformed_mat[3], transformed_mat[0]);
}

void TransformMatrix2D::transform(Velocity2D& vel) const
{
    float temp_x = (mat_[0] * vel.x) + (mat_[1] * vel.y) + (mat_[5] * vel.theta);
    float temp_y = (mat_[3] * vel.x) + (mat_[4] * vel.y) - (mat_[2] * vel.theta);
    vel.x = temp_x;
    vel.y = temp_y;
}

void TransformMatrix2D::transform(LineSegment2D& line_segment) const
{
    transform(line_segment.start);
    transform(line_segment.end);
}

void TransformMatrix2D::transform(Polyline2D& polyline) const
{
    for ( Point2D& vertex : polyline.vertices )
    {
        transform(vertex);
    }
}

void TransformMatrix2D::transform(PointCloud2D& cloud) const
{
    for ( Point2D& pt : cloud )
    {
        transform(pt);
    }
}

void TransformMatrix2D::transform(Path& pose_path) const
{
    for ( Pose2D& pose : pose_path )
    {
        transform(pose);
    }
}

TransformMatrix2D& TransformMatrix2D::operator = (const TransformMatrix2D& other)
{
    mat_ = other.mat_;
    return *this;
}

TransformMatrix2D TransformMatrix2D::operator * (const TransformMatrix2D& tf_mat) const
{
    TransformMatrix2D result_tf_mat(*this);
    result_tf_mat *= tf_mat;
    return result_tf_mat;
}

TransformMatrix2D& TransformMatrix2D::operator *= (const TransformMatrix2D& tf_mat)
{
    float arr[6];
    arr[0] = (mat_[0] * tf_mat[0]) + (mat_[1] * tf_mat[3]);
    arr[1] = (mat_[0] * tf_mat[1]) + (mat_[1] * tf_mat[4]);
    arr[2] = (mat_[0] * tf_mat[2]) + (mat_[1] * tf_mat[5]) + mat_[2];
    arr[3] = (mat_[3] * tf_mat[0]) + (mat_[4] * tf_mat[3]);
    arr[4] = (mat_[3] * tf_mat[1]) + (mat_[4] * tf_mat[4]);
    arr[5] = (mat_[3] * tf_mat[2]) + (mat_[4] * tf_mat[5]) + mat_[5];
    for ( size_t i = 0; i < 6; i++ )
    {
        mat_[i] = arr[i];
    }
    return *this;
}

Point2D TransformMatrix2D::operator * (const Point2D& point) const
{
    Point2D transformed_point(point);
    transform(transformed_point);
    return transformed_point;
}

Circle TransformMatrix2D::operator * (const Circle& circle) const
{
    Circle transformed_circle(circle);
    transform(transformed_circle);
    return transformed_circle;
}

Pose2D TransformMatrix2D::operator * (const Pose2D& pose) const
{
    TransformMatrix2D transformed_mat = (*this) * pose.asMat();
    return transformed_mat.asPose2D();
}

Velocity2D TransformMatrix2D::operator * (const Velocity2D& vel) const
{
    Velocity2D transformed_vel(vel);
    transform(transformed_vel);
    return transformed_vel;
}

LineSegment2D TransformMatrix2D::operator * (const LineSegment2D& line_segment) const
{
    LineSegment2D transformed_line_segment(line_segment);
    transform(transformed_line_segment);
    return transformed_line_segment;
}

Polyline2D TransformMatrix2D::operator * (const Polyline2D& polyline) const
{
    Polyline2D transformed_polyline(polyline);
    transform(transformed_polyline);
    return transformed_polyline;
}

Polygon2D TransformMatrix2D::operator * (const Polygon2D& polygon) const
{
    Polyline2D transformed_polygon(polygon);
    transform(transformed_polygon);
    return transformed_polygon;
}

Path TransformMatrix2D::operator * (const Path& pose_path) const
{
    Path transformed_path(pose_path);
    transform(transformed_path);
    return transformed_path;
}

const float& TransformMatrix2D::operator [] (unsigned int index) const
{
    return mat_[index];
}

bool TransformMatrix2D::operator == (const TransformMatrix2D& other) const
{
    for ( size_t i = 0; i < mat_.size(); i++ )
    {
        if ( std::fabs(mat_[i] - other[i]) > 1e-3f )
        {
            return false;
        }
    }
    return true;
}

bool TransformMatrix2D::operator != (const TransformMatrix2D& other) const
{
    return !((*this) == other);
}

std::ostream& operator << (std::ostream& out, const TransformMatrix2D& tf_mat)
{
    out << std::setprecision(3) << std::fixed;
    out << tf_mat.mat_[0] << "\t"
        << tf_mat.mat_[1] << "\t"
        << tf_mat.mat_[2] << std::endl
        << tf_mat.mat_[3] << "\t"
        << tf_mat.mat_[4] << "\t"
        << tf_mat.mat_[5] << std::endl
        << "0.000\t0.000\t1.000";
    return out;
}

} // namespace geometry_common
} // namespace kelo
