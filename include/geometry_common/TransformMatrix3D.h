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

#ifndef KELO_GEOMETRY_COMMON_TRANSFORM_MATRIX_3D_H
#define KELO_GEOMETRY_COMMON_TRANSFORM_MATRIX_3D_H

#include <array>

#include <tf/transform_datatypes.h>

namespace kelo
{
namespace geometry_common
{

// Forward declaration 
class Point3D;
using Vector3D = Point3D;

/**
 * @brief Transformation matrix for three dimensional space
 * 
 */
class TransformMatrix3D
{
    public:

        using Ptr = std::shared_ptr<TransformMatrix3D>;
        using ConstPtr = std::shared_ptr<const TransformMatrix3D>;

        TransformMatrix3D():
            TransformMatrix3D(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f) {}

        TransformMatrix3D(float x, float y, float z,
                       float roll, float pitch, float yaw);

        TransformMatrix3D(float x, float y, float z,
                       float qx, float qy, float qz, float qw);

        TransformMatrix3D(const tf::StampedTransform& stamped_transform);

        TransformMatrix3D(const TransformMatrix3D& tf_mat);

        virtual ~TransformMatrix3D() {};

        void update(float x, float y, float z,
                    float roll, float pitch, float yaw);

        void update(float x, float y, float z,
                    float qx, float qy, float qz, float qw);

        void update(const TransformMatrix3D& tf_mat);

        void updateX(float x);

        void updateY(float y);

        void updateZ(float z);

        void updateRoll(float roll);

        void updatePitch(float pitch);

        void updateYaw(float yaw);

        void updateRollPitchYaw(float roll, float pitch, float yaw);

        void updateQuaternion(float qx, float qy, float qz, float qw);

        TransformMatrix3D calcInverse() const;

        void invert();

        float x() const;

        float y() const;

        float z() const;

        float roll() const;

        float pitch() const;

        float yaw() const;

        std::array<float, 4> quaternion() const;

        std::array<float, 9> rotationMatrix() const;

        Vector3D translationVector() const;

        void transform(Point3D& point) const;

        /**
         * @brief 
         * 
         * @param other 
         * @return TransformMatrix3D& 
         */
        TransformMatrix3D& operator = (const TransformMatrix3D& other);

        TransformMatrix3D operator * (const TransformMatrix3D& tf_mat) const;

        TransformMatrix3D& operator *= (const TransformMatrix3D& tf_mat);

        Point3D operator * (const Point3D& point) const;

        const float& operator [] (unsigned int index) const;

        /**
         * @brief 
         *
         * @param tf_mat
         * @return 
         */
        bool operator == (const TransformMatrix3D& tf_mat) const;

        /**
         * @brief 
         * 
         * @param out 
         * @param tf_mat 
         * @return std::ostream& 
         */
        friend std::ostream& operator << (
                std::ostream& out,
                const TransformMatrix3D& tf_mat);

    protected:
        std::array<float, 12> mat_;

};

} // namespace geometry_common
} // namespace kelo
#endif // KELO_GEOMETRY_COMMON_TRANSFORM_MATRIX_3D_H
