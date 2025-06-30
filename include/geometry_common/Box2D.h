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
#include "geometry_common/Polygon2D.h"

namespace kelo
{
namespace geometry_common
{

/**
 * @brief Axis aligned rectangle
 *
 */
class Box2D
{
    public:
        float min_x, max_x, min_y, max_y;

        using Ptr = std::shared_ptr<Box2D>;
        using ConstPtr = std::shared_ptr<const Box2D>;

        /**
         * @brief Default c-tor
         *
         * @param _min_x minimum boundary on X-axis
         * @param _max_x maximum boundary on X-axis
         * @param _min_y minimum boundary on Y-axis
         * @param _max_y maximum boundary on Y-axis
         */
        Box2D(float _min_x = 0.0f, float _max_x = 0.0f,
            float _min_y = 0.0f, float _max_y = 0.0f):
            min_x(_min_x), max_x(_max_x),
            min_y(_min_y), max_y(_max_y) {};

        /**
         * @brief Copy c-tor
         *
         * @param box Box2D object to copy from
         */
        Box2D(const Box2D& box):
            min_x(box.min_x), max_x(box.max_x),
            min_y(box.min_y), max_y(box.max_y) {};

        /**
         * @brief Bounding box from vector of points
         *
         * @param points vector of points whose bounding box gets calculated
         */
        Box2D(const PointVec2D& points);

        /**
         * @brief Bounding box of a polygon
         *
         * @param polygon polygon object whose bounding box gets calculated
         */
        Box2D(const Polygon2D& polygon):
            Box2D(polygon.vertices) {};

        /**
         * @brief d-tor
         *
         */
        virtual ~Box2D() {};

        /**
         * @brief get size of box in X-axis
         *
         * @return size of box in X-axis
         */
        float sizeX() const;

        /**
         * @brief get size of box in Y-axis
         *
         * @return size of box in Y-axis
         */
        float sizeY() const;

        /**
         * @brief get the center of box
         *
         * @return center as Point2D
         */
        Point2D center() const;

        /**
         * @brief Get an RViz visualization marker for the box object
         *
         * @param frame The frame in which the box is specified
         * @param red The red color-component to be used in the marker
         * color in the range [0.0, 1.0]
         * @param green The green color-component to be used in the marker
         * color in the range [0.0, 1.0]
         * @param blue The blue color-component to be used in the marker
         * color in the range [0.0, 1.0]
         * @param alpha The transparency of the generated marker
         * in the range [0.0, 1.0]
         * @return visualization_msgs::msg::Marker A marker object representing the
         * box
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
        Polygon2D asPolygon2D() const;

        /**
         * @brief Check if a 2D point lies within the Box.
         *
         * @param p 2D point to be checked
         * @return bool True if the point lies inside the box, false otherwise
         */
        bool containsPoint(const Point2D& p) const;

        /**
         * @brief assignment operator overload
         *
         * @param other rhs Box2D object
         * @return Box2D& lhs Box2D object
         */
        Box2D& operator = (const Box2D& other);

        /**
         * @brief equality operator overload
         *
         * @param other rhs Box2D object
         * @return bool true if both Box2D are almost equal; false otherwise
         */
        bool operator == (const Box2D& other) const;

        /**
         * @brief inequality operator overload
         *
         * @param other rhs Box2D object
         * @return bool false if both Box2D are almost equal; true otherwise
         */
        bool operator != (const Box2D& other) const;

        /**
         * @brief << operator overload
         *
         * @param out The stream object to which the box information should be appended
         * @param box The box whose data should be appended to the stream object
         * @return std::ostream& The stream object representing the concatenation
         * of the input stream and the box information
         */
        friend std::ostream& operator << (std::ostream& out, const Box2D& box);

};

} // namespace geometry_common
} // namespace kelo
