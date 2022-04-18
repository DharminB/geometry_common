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

#ifndef KELO_GEOMETRY_COMMON_POINT_2D_H
#define KELO_GEOMETRY_COMMON_POINT_2D_H

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>

#include <cmath>

namespace kelo
{
namespace geometry_common
{

// Forward declaration 
class Pose2D;

/**
 * @brief Point for two dimensional space
 * 
 */
class Point2D
{
    public:
        float x{0.0f}, y{0.0f};

        using Ptr = std::shared_ptr<Point2D>;
        using ConstPtr = std::shared_ptr<const Point2D>;

        /**
         * @brief
         * 
         * @param _x 
         * @param _y 
         */
        Point2D(float _x = 0.0f, float _y = 0.0f):
            x(_x),
            y(_y) {}

        /**
         * @brief
         * 
         * @param point 
         */
        Point2D(const Point2D& point):
            Point2D(point.x, point.y) {}

        /**
         * @brief Construct Point2D from geometry_msgs::PointStamped object.
         * Ignores z coordinate.
         *
         * @param point geometry_msgs::PointStamped object
         */
        Point2D(const geometry_msgs::PointStamped& point):
            Point2D(point.point) {}

        /**
         * @brief Construct Point2D from geometry_msgs::Point object. Ignores z
         * coordinate.
         *
         * @param point geometry_msgs::Point object
         */
        Point2D(const geometry_msgs::Point& point):
            Point2D(point.x, point.y) {}

        /**
         * @brief Construct Point2D from geometry_msgs::Point32 object. Ignores
         * z coordinate.
         *
         * @param point geometry_msgs::Point32 object
         */
        Point2D(const geometry_msgs::Point32& point):
            Point2D(point.x, point.y) {}

        /**
         * @brief
         * 
         */
        virtual ~Point2D() {}

        /**
         * @brief
         * 
         * @return geometry_msgs::Point 
         */
        geometry_msgs::Point asPoint() const;

        /**
         * @brief
         * 
         * @return geometry_msgs::Point32 
         */
        geometry_msgs::Point32 asPoint32() const;

        /**
         * @brief
         * 
         * @param frame 
         * @return geometry_msgs::PointStamped 
         */
        geometry_msgs::PointStamped asPointStamped(
                const std::string& frame = "map") const;

        /**
         * @brief
         * 
         * @param p 
         * @return float 
         */
        inline float distTo(const Point2D& p) const
        {
            return std::sqrt(squaredDistTo(p));
        };

        /**
         * @brief
         * 
         * @param p 
         * @return float 
         */
        inline float squaredDistTo(const Point2D& p) const
        {
            return std::pow(x - p.x, 2) + std::pow(y - p.y, 2);
        };

        /**
         * @brief 
         * 
         * @return float 
         */
        inline float magnitude() const
        {
            return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        };

        /**
         * @brief Normalise vector in place
         * 
         */
        void normalise();

        /**
         * @brief: Return a normalised vector
         *
         * @return: Point2D
         */
        Point2D asNormalised() const;

        /**
         * @brief 
         * 
         * @param point 
         * @return float 
         */
        float scalarCrossProduct(const Point2D& point) const;

        /**
         * @brief calculates dot product of two 2D vectors
         *
         * @param point Second vector
         *
         * @return float dot product
         */
        float dotProduct(const Point2D& point) const;

        /**
         * @brief Return angle of vector/point w.r.t. origin point
         *
         * @return angle
         */
        float angle() const;

        /**
         * @brief
         * 
         * @param frame 
         * @param red 
         * @param green 
         * @param blue 
         * @param alpha 
         * @param diameter 
         * @param z 
         * @return visualization_msgs::Marker 
         */
        visualization_msgs::Marker asMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float diameter = 0.2f,
                float z = 0.0f) const;

        /**
         * @brief 
         * 
         * @param other 
         * @return Point2D& 
         */
        Point2D& operator = (const Point2D& other);

        /**
         * @brief 
         * 
         * @param other 
         * @return Point2D 
         */
        Point2D operator - (const Point2D& other) const;

        /**
         * @brief 
         * 
         * @param other 
         * @return Point2D 
         */
        Point2D operator + (const Point2D& other) const;

        /**
         * @brief Scale point with a constant scalar number
         * 
         * @param scalar number the point will be scaled with
         * @return Point2D 
         */
        Point2D operator * (float scalar) const;

        /**
         * @brief 
         * 
         * @param other 
         * @return bool 
         */
        bool operator == (const Point2D& other) const;

        /**
         * @brief 
         * 
         * @param out 
         * @param point 
         * @return std::ostream& 
         */
        friend std::ostream& operator << (std::ostream& out, const Point2D& point);

};

/**
 * @brief Mathematical two dimensional vector
 * 
 */
using Vector2D = Point2D;

/**
 * @brief Collection (std::vector) of Point2D objects
 * 
 */
using PointVec2D = std::vector<Point2D>;

/**
 * @brief
 * 
 */
using PointCloud2D = std::vector<Point2D>;

} // namespace geometry_common
} // namespace kelo
#endif // KELO_GEOMETRY_COMMON_POINT_H
