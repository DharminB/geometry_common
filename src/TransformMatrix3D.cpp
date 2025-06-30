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
#include "geometry_common/TransformMatrix3D.h"

namespace kelo
{
namespace geometry_common
{

TransformMatrix3D::TransformMatrix3D(
        float x, float y, float z, float roll, float pitch, float yaw)
{
    update(x, y, z, roll, pitch, yaw);
}

TransformMatrix3D::TransformMatrix3D(
        float x, float y, float z, float qx, float qy, float qz, float qw)
{
    update(x, y, z, qx, qy, qz, qw);
}

TransformMatrix3D::TransformMatrix3D(const geometry_msgs::msg::TransformStamped& ts)
{
    update(ts);
}

TransformMatrix3D::TransformMatrix3D(const TransformMatrix3D& tf_mat)
{
    update(tf_mat);
}

void TransformMatrix3D::update(
        float x, float y, float z, float roll, float pitch, float yaw)
{
    updateX(x);
    updateY(y);
    updateZ(z);
    updateRollPitchYaw(roll, pitch, yaw);
}

void TransformMatrix3D::update(
        float x, float y, float z, float qx, float qy, float qz, float qw)
{
    updateX(x);
    updateY(y);
    updateZ(z);
    updateQuaternion(qx, qy, qz, qw);
}

void TransformMatrix3D::update(const geometry_msgs::msg::TransformStamped& ts)
{
    update(
        ts.transform.translation.x,
        ts.transform.translation.y,
        ts.transform.translation.z,
        ts.transform.rotation.x,
        ts.transform.rotation.y,
        ts.transform.rotation.z,
        ts.transform.rotation.w);
}

void TransformMatrix3D::update(const TransformMatrix3D& tf_mat)
{
    for ( size_t i = 0; i < 12; i++ )
    {
        mat_[i] = tf_mat[i];
    }
}

void TransformMatrix3D::updateX(float x)
{
    mat_[3] = x;
}

void TransformMatrix3D::updateY(float y)
{
    mat_[7] = y;
}

void TransformMatrix3D::updateZ(float z)
{
    mat_[11] = z;
}

void TransformMatrix3D::updateRoll(float roll)
{
    updateRollPitchYaw(roll, pitch(), yaw());
}

void TransformMatrix3D::updatePitch(float pitch)
{
    updateRollPitchYaw(roll(), pitch, yaw());
}

void TransformMatrix3D::updateYaw(float yaw)
{
    updateRollPitchYaw(roll(), pitch(), yaw);
}

void TransformMatrix3D::updateRollPitchYaw(float roll, float pitch, float yaw)
{
    mat_[0] = (std::cos(yaw) * std::cos(pitch));
    mat_[1] = (std::cos(yaw) * std::sin(pitch) * std::sin(roll)) -
              (std::sin(yaw) * std::cos(roll));
    mat_[2] = (std::cos(yaw) * std::sin(pitch) * std::cos(roll)) +
              (std::sin(yaw) * std::sin(roll));
    mat_[4] = (std::sin(yaw) * std::cos(pitch));
    mat_[5] = (std::sin(yaw) * std::sin(pitch) * std::sin(roll)) +
              (std::cos(yaw) * std::cos(roll));
    mat_[6] = (std::sin(yaw) * std::sin(pitch) * std::cos(roll)) -
              (std::cos(yaw) * std::sin(roll));
    mat_[8] = -std::sin(pitch);
    mat_[9] = std::cos(pitch) * std::sin(roll);
    mat_[10] = std::cos(pitch) * std::cos(roll);
}

void TransformMatrix3D::updateQuaternion(float qx, float qy, float qz, float qw)
{
    /**
     * source: https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
     */
    mat_[0] = 2 * (qw * qw + qx * qx) - 1;
    mat_[1] = 2 * (qx * qy - qw * qz);
    mat_[2] = 2 * (qx * qz + qw * qy);
    mat_[4] = 2 * (qx * qy + qw * qz);
    mat_[5] = 2 * (qw * qw + qy * qy) - 1;
    mat_[6] = 2 * (qy * qz - qw * qx);
    mat_[8] = 2 * (qx * qz - qw * qy);
    mat_[9] = 2 * (qy * qz + qw * qx);
    mat_[10] = 2 * (qw * qw + qz * qz) - 1;
}

TransformMatrix3D TransformMatrix3D::calcInverse() const
{
    TransformMatrix3D inv_tf_mat(*this);
    inv_tf_mat.invert();
    return inv_tf_mat;
}

void TransformMatrix3D::invert()
{
    // taking transpose of rotation matrix part since
    // inverse(M) == transpose(M) : where M is a orthonormal rotation matrix
    for ( size_t i = 0; i < 2; i++ )
    {
        for ( size_t j = i+1; j < 3; j++ )
        {
            float temp = mat_[(i*4)+j];
            mat_[(i*4)+j] = mat_[(j*4)+i];
            mat_[(j*4)+i] = temp;
        }
    }

    // - inverse(M) * T : where M is as above and T is translation vector
    float x = mat_[3];
    float y = mat_[7];
    float z = mat_[11];
    mat_[3] = -((mat_[0] * x) + (mat_[1] * y) + (mat_[2] * z));
    mat_[7] = -((mat_[4] * x) + (mat_[5] * y) + (mat_[6] * z));
    mat_[11] = -((mat_[8] * x) + (mat_[9] * y) + (mat_[10] * z));
}

float TransformMatrix3D::x() const
{
    return mat_[3];
}

float TransformMatrix3D::y() const
{
    return mat_[7];
}

float TransformMatrix3D::z() const
{
    return mat_[11];
}

float TransformMatrix3D::roll() const
{
    return std::atan2(mat_[9], mat_[10]);
}

float TransformMatrix3D::pitch() const
{
    return std::atan2(-mat_[8], std::sqrt(pow(mat_[9], 2) + pow(mat_[10], 2)));
}

float TransformMatrix3D::yaw() const
{
    return std::atan2(mat_[4], mat_[0]);
}

std::array<float, 4> TransformMatrix3D::quaternion() const
{
    std::array<float, 4> q;
    Utils::convertEulerToQuaternion(roll(), pitch(), yaw(), q[0], q[1], q[2], q[3]);
    return q;
}

std::array<float, 9> TransformMatrix3D::rotationMatrix() const
{
    std::array<float, 9> rot_mat;
    for ( size_t i = 0; i < 3; i++ )
    {
        for ( size_t j = 0; j < 3; j++ )
        {
            rot_mat[(i*3)+j] = mat_[(i*4) + j];
        }
    }
    return rot_mat;
}

Vector3D TransformMatrix3D::translationVector() const
{
    return Vector3D(mat_[3], mat_[7], mat_[11]);
}

void TransformMatrix3D::transform(Point3D& point) const
{
    float temp_x = (mat_[0] * point.x) + (mat_[1] * point.y) +
                   (mat_[2] * point.z) + mat_[3];
    float temp_y = (mat_[4] * point.x) + (mat_[5] * point.y) +
                   (mat_[6] * point.z) + mat_[7];
    float temp_z = (mat_[8] * point.x) + (mat_[9] * point.y) +
                   (mat_[10] * point.z) + mat_[11];
    point.x = temp_x;
    point.y = temp_y;
    point.z = temp_z;
}

void TransformMatrix3D::transform(PointCloud3D& cloud) const
{
    for ( Point3D& pt : cloud )
    {
        transform(pt);
    }
}

TransformMatrix3D& TransformMatrix3D::operator = (const TransformMatrix3D& other)
{
    mat_ = other.mat_;
    return *this;
}

TransformMatrix3D TransformMatrix3D::operator * (const TransformMatrix3D& tf_mat) const
{
    TransformMatrix3D result_tf_mat(*this);
    result_tf_mat *= tf_mat;
    return result_tf_mat;
}

TransformMatrix3D& TransformMatrix3D::operator *= (const TransformMatrix3D& tf_mat)
{
    float arr[12] = {0.0f}; // temporary array

    // apply rotation to tf_mat by multipling 3x3 rot mat part of mat_ with tf_mat
    for ( size_t i = 0; i < 3; i++ )
    {
        for ( size_t j = 0; j < 4; j++ )
        {
            for ( size_t k = 0; k < 3; k++ )
            {
                arr[i*4 + j] += mat_[i*4 + k] * tf_mat[k*4 + j];
            }
        }
    }

    // apply translation to tf_mat by adding 3x1 translation part of mat_
    arr[3] += mat_[3];
    arr[7] += mat_[7];
    arr[11] += mat_[11];

    //copy temp arr to mat_
    for ( size_t i = 0; i < 12; i++ )
    {
        mat_[i] = arr[i];
    }
    return *this;
}

Point3D TransformMatrix3D::operator * (const Point3D& point) const
{
    Point3D transformed_point(point);
    transform(transformed_point);
    return transformed_point;
}

PointCloud3D TransformMatrix3D::operator * (const PointCloud3D& cloud) const
{
    PointCloud3D transformed_cloud(cloud);
    transform(transformed_cloud);
    return transformed_cloud;
}

const float& TransformMatrix3D::operator [] (unsigned int index) const
{
    return mat_[index];
}

bool TransformMatrix3D::operator == (const TransformMatrix3D& other) const
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

bool TransformMatrix3D::operator != (const TransformMatrix3D& other) const
{
    return !((*this) == other);
}

std::ostream& operator << (std::ostream& out, const TransformMatrix3D& tf_mat)
{
    out << std::setprecision(3) << std::fixed;
    out << tf_mat.mat_[0] << "\t"
        << tf_mat.mat_[1] << "\t"
        << tf_mat.mat_[2] << "\t"
        << tf_mat.mat_[3] << std::endl
        << tf_mat.mat_[4] << "\t"
        << tf_mat.mat_[5] << "\t"
        << tf_mat.mat_[6] << "\t"
        << tf_mat.mat_[7] << std::endl
        << tf_mat.mat_[8] << "\t"
        << tf_mat.mat_[9] << "\t"
        << tf_mat.mat_[10] << "\t"
        << tf_mat.mat_[11] << std::endl
        << "0.000\t0.000\t0.000\t1.000";
    return out;
}

} // namespace geometry_common
} // namespace kelo
