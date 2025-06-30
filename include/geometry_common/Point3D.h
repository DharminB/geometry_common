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

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "geometry_common/Point2D.h"

namespace kelo
{
namespace geometry_common
{

/**
 * @brief Point for three dimensional space
 * 
 */
class Point3D
{
    public:
        float x{0.0f}, y{0.0f}, z{0.0f};

        using Ptr = std::shared_ptr<Point3D>;
        using ConstPtr = std::shared_ptr<const Point3D>;

        /**
         * @brief
         * 
         * @param _x 
         * @param _y 
         * @param _z 
         */
        Point3D(float _x = 0.0f, float _y = 0.0f, float _z = 0.0f):
            x(_x),
            y(_y),
            z(_z) {}

        /**
         * @brief
         * 
         * @param point 
         * @param _z 
         */
        Point3D(const Point2D& point, float _z = 0.0f):
            Point3D(point.x, point.y, _z) {}

        /**
         * @brief
         * 
         * @param point 
         */
        Point3D(const Point3D& point):
            Point3D(point.x, point.y, point.z) {}

        /**
         * @brief
         * 
         * @param point 
         */
        Point3D(const geometry_msgs::msg::PointStamped& point):
            Point3D(point.point) {}

        /**
         * @brief
         * 
         * @param point 
         */
        Point3D(const geometry_msgs::msg::Point& point):
            Point3D(point.x, point.y, point.z) {}

        /**
         * @brief
         * 
         * @param point 
         */
        Point3D(const geometry_msgs::msg::Point32& point):
            Point3D(point.x, point.y, point.z) {}

        /**
         * @brief
         * 
         */
        virtual ~Point3D() {}

        /**
         * @brief
         * 
         * @return geometry_msgs::msg::Point 
         */
        geometry_msgs::msg::Point asPoint() const;

        /**
         * @brief
         * 
         * @return geometry_msgs::msg::Point32 
         */
        geometry_msgs::msg::Point32 asPoint32() const;

        /**
         * @brief
         * 
         * @param frame 
         * @return geometry_msgs::msg::PointStamped 
         */
        geometry_msgs::msg::PointStamped asPointStamped(
                const std::string& frame = "map") const;

        /**
         * @brief
         * 
         * @param p 
         * @return float 
         */
        inline float distTo(const Point3D& p) const
        {
            return std::sqrt(squaredDistTo(p));
        };

        /**
         * @brief
         * 
         * @param p 
         * @return float 
         */
        inline float squaredDistTo(const Point3D& p) const
        {
            return std::pow(x - p.x, 2) + std::pow(y - p.y, 2) + std::pow(z - p.z, 2);
        };

        /**
         * @brief 
         * 
         * @return float 
         */
        inline float magnitude() const
        {
            return std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2));
        };

        /**
         * @brief Normalise vector in place
         * 
         */
        void normalise();

        /**
         * @brief: Return a normalised vector
         *
         * @return: Point3D
         */
        Point3D asNormalised() const;

        /**
         * @brief Calculate dot product of two 3D vectors
         *
         * @param point Second vector
         *
         * @return float dot product
         */
        float dotProduct(const Point3D& point) const;

        /**
         * @brief
         * 
         * @param frame 
         * @param red 
         * @param green 
         * @param blue 
         * @param alpha 
         * @param diameter 
         * @return visualization_msgs::msg::Marker 
         */
        visualization_msgs::msg::Marker asMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float diameter = 0.2f) const;

        /**
         * @brief 
         * 
         * @param other 
         * @return Point3D& 
         */
        Point3D& operator = (const Point3D& other);

        /**
         * @brief 
         * 
         * @param other 
         * @return Point3D 
         */
        Point3D operator - (const Point3D& other) const;

        /**
         * @brief 
         * 
         * @param other
         * @return Point3D 
         */
        Point3D operator + (const Point3D& other) const;

        /**
         * @brief Scale point with a constant scalar number
         * 
         * @param scalar number the point will be scaled with
         * @return Point3D 
         */
        Point3D operator * (float scalar) const;

        /**
         * @brief Inversely scale point with a constant scalar number
         *
         * @param scalar number the point will be inversely scaled with
         * @return Point3D
         */
        Point3D operator / (float scalar) const;

        /**
         * @brief Equality checking operator overload. Checks if the members are
         * almost equal by checking if difference is smaller than threshold
         *
         * @param other rhs Point3D object
         * @return bool true is all members are almost equal; false otherwise
         */
        bool operator == (const Point3D& other) const;

        /**
         * @brief Inequality checking operator overload.
         *
         * @param other rhs Point3D object
         * @return bool false is all members are almost equal; true otherwise
         */
        bool operator != (const Point3D& other) const;

        /**
         * @brief 
         * 
         * @param out 
         * @param point 
         * @return std::ostream& 
         */
        friend std::ostream& operator << (std::ostream& out, const Point3D& point);

};

/**
 * @brief Mathematical three dimensional vector
 * 
 */
using Vector3D = Point3D;

/**
 * @brief Collection (std::vector) of Point3D objects
 * 
 */
using PointVec3D = std::vector<Point3D>;

/**
 * @brief 
 * 
 */
using PointCloud3D = std::vector<Point3D>;

} // namespace geometry_common
} // namespace kelo
