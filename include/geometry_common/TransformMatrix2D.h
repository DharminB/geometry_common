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

#pragma once

#include <array>

#include "geometry_msgs/msg/transform_stamped.hpp"

namespace kelo
{
namespace geometry_common
{

// Forward declaration 
class Pose2D;
class Point2D;
class Circle;
class LineSegment2D;
class Polyline2D;
class Polygon2D;
class XYTheta;
using PointCloud2D = std::vector<Point2D>;
using Vector2D = Point2D;
using Velocity2D = XYTheta;
using Path = std::vector<Pose2D>;

/**
 * @brief Transformation matrix for two dimensional space
 * 
 */
class TransformMatrix2D
{
    public:

        using Ptr = std::shared_ptr<TransformMatrix2D>;
        using ConstPtr = std::shared_ptr<const TransformMatrix2D>;

        TransformMatrix2D():
            TransformMatrix2D(0.0f, 0.0f, 0.0f) {}

        /**
         * @brief Construct transformation matrix with euler angle values
         *
         * @param x Translation in X axis
         * @param y Translation in Y axis
         * @param theta Rotation on Z axis
         */
        TransformMatrix2D(float x, float y, float theta);

        /**
         * @brief Construct transformation matrix with quaternion angle values
         *
         * @param x
         * @param y
         * @param qx
         * @param qy
         * @param qz
         * @param qw
         */
        TransformMatrix2D(float x, float y, float qx, float qy, float qz, float qw);

        TransformMatrix2D(const geometry_msgs::msg::TransformStamped& ts);

        TransformMatrix2D(const Pose2D& pose);

        TransformMatrix2D(const XYTheta& x_y_theta);

        TransformMatrix2D(const TransformMatrix2D& tf_mat);

        /**
         * @brief
         * 
         */
        virtual ~TransformMatrix2D() {}

        void update(float x, float y, float theta);

        void update(float x, float y, float qx, float qy, float qz, float qw);

        void update(const geometry_msgs::msg::TransformStamped& ts);

        void update(const Pose2D& pose);

        void update(const XYTheta& x_y_theta);

        void update(const TransformMatrix2D& tf_mat);

        void updateX(float x);

        void updateY(float y);

        void updateTheta(float theta);

        void updateQuaternion(float qx, float qy, float qz, float qw);

        TransformMatrix2D calcInverse() const;

        void invert();

        float x() const;

        float y() const;

        float theta() const;

        std::array<float, 4> quaternion() const;

        std::array<float, 4> rotationMatrix() const;

        Vector2D translationVector() const;

        Pose2D asPose2D() const;

        void transform(Point2D& point) const;

        void transform(Circle& circle) const;

        void transform(Pose2D& pose) const;

        /**
         * @brief transform two dimensional velocity in place
         *
         * [ cos(theta)  -sin(theta)  y ]   [ vel_x     ]    [ target_vel_x ]
         * [ sin(theta)   cos(theta) -x ] * [ vel_y     ] => [ target_vel_y ]
         * [ 0            0           1 ]   [ vel_theta ]    [ target_vel_theta ]
         *
         * @param vel velocity that needs to be transformed in place
         */
        void transform(Velocity2D& vel) const;

        void transform(LineSegment2D& line_segment) const;

        void transform(Polyline2D& polyline) const;

        void transform(PointCloud2D& cloud) const;

        void transform(Path& pose_path) const;

        /**
         * @brief 
         * 
         * @param other 
         * @return TransformMatrix2D& 
         */
        TransformMatrix2D& operator = (const TransformMatrix2D& other);

        TransformMatrix2D operator * (const TransformMatrix2D& tf_mat) const;

        TransformMatrix2D& operator *= (const TransformMatrix2D& tf_mat);

        Point2D operator * (const Point2D& vec) const;

        Circle operator * (const Circle& circle) const;

        Pose2D operator * (const Pose2D& vec) const;

        Velocity2D operator * (const Velocity2D& vec) const;

        LineSegment2D operator * (const LineSegment2D& line_segment) const;

        Polyline2D operator * (const Polyline2D& polyline) const;

        Polygon2D operator * (const Polygon2D& polygon) const;

        Path operator * (const Path& pose_path) const;

        const float& operator [] (unsigned int index) const;

        /**
         * @brief Equality checking operator overload. Checks if the members are
         * almost equal by checking if difference is smaller than threshold
         *
         * @param other rhs TransformMatrix2D object
         * @return bool true is all members are almost equal; false otherwise
         */
        bool operator == (const TransformMatrix2D& other) const;

        /**
         * @brief Inequality checking operator overload.
         *
         * @param other rhs TransformMatrix2D object
         * @return bool false is all members are almost equal; true otherwise
         */
        bool operator != (const TransformMatrix2D& other) const;

        /**
         * @brief 
         * 
         * @param out 
         * @param tf_mat 
         * @return std::ostream& 
         */
        friend std::ostream& operator << (
                std::ostream& out,
                const TransformMatrix2D& tf_mat);

    protected:
        std::array<float, 6> mat_;

};

} // namespace geometry_common
} // namespace kelo
