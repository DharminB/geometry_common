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

#include "visualization_msgs/msg/marker.hpp"

#include "geometry_common/Point2D.h"

namespace kelo
{
namespace geometry_common
{

// Forward declaration 
class Polygon2D;

/**
 * @brief Circle with center (as Point2D) and radius
 */
class Circle : public Point2D
{
    public:
        /// radius of the circle
        float r{0.001f};

        /// std::shared_ptr of Circle object
        using Ptr = std::shared_ptr<Circle>;

        /// std::shared_ptr of const Circle object
        using ConstPtr = std::shared_ptr<const Circle>;

        /**
         * @brief default constructor
         *
         * @param _x circle center's x coordinate
         * @param _y circle center's y coordinate
         * @param _r circle's radius
         */
        Circle(float _x = 0.0f, float _y = 0.0f, float _r = 0.001f):
            Point2D(_x, _y), r(_r) {}

        /**
         * @brief Copy constructor
         *
         * @param circle original circle from which a copy needs to be made
         */
        Circle(const Circle& circle):
            Circle(circle.x, circle.y, circle.r) {}

        /**
         * @brief Construct a Circle centered at given point p with given radius
         *
         * @param center center point of Circle
         * @param _r radius of Circle
         */
        Circle(const Point2D& center, float _r = 0.001f):
            Point2D(center), r(_r) {}

        /**
         * @brief default d-tor
         */
        virtual ~Circle() = default;

        /**
         * @brief get the center of circle as a Point2D object
         *
         * @return center of Circle object
         */
        Point2D center() const;

        /**
         * @brief set center of Circle
         *
         * @param center new center represeted as a Point2D
         */
        void setCenter(const Point2D& center);

        /**
         * @brief calculate a circle such that points p1, p2 and p3 are lying on
         * it
         * source: https://stackoverflow.com/a/57406014/10460994
         *
         * @param p1
         * @param p2
         * @param p3
         * @param circle
         *
         * @return false if the points are collinear; true otherwise
         */
        static bool fromPoints(const Point2D& p1, const Point2D& p2,
                               const Point2D& p3, Circle& circle);

        /**
         * @brief
         * 
         * @param frame 
         * @param red 
         * @param green 
         * @param blue 
         * @param alpha 
         * @return visualization_msgs::msg::Marker 
         */
        visualization_msgs::msg::Marker asMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f) const;

        /**
         * @brief return boundary as Polygon2D object
         *
         * @return resulting Polygon2D object
         */
        Polygon2D asPolygon2D(size_t num_of_segments = 16) const;

        /**
         * @brief 
         * 
         * @param other 
         * @return Point2D& 
         */
        Circle& operator = (const Circle& other);

        /**
         * @brief Equality checking operator overload. Checks if the members are
         * almost equal by checking if difference is smaller than threshold
         *
         * @param other rhs Circle object
         * @return bool true is all members are almost equal; false otherwise
         */
        bool operator == (const Circle& other) const;

        /**
         * @brief Inequality checking operator overload.
         *
         * @param other rhs Circle object
         * @return bool false is all members are almost equal; true otherwise
         */
        bool operator != (const Circle& other) const;

        /**
         * @brief 
         * 
         * @param out 
         * @param circle 
         * @return std::ostream& 
         */
        friend std::ostream& operator << (std::ostream& out, const Circle& circle);

};

} // namespace geometry_common
} // namespace kelo
