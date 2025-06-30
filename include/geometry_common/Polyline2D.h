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

#include "geometry_common/LineSegment2D.h"
#include "geometry_common/Pose2D.h"

namespace kelo
{
namespace geometry_common
{

// Forward declaration
class LineSegment2D;

/**
 * @brief Represents a polyline (or a linestring) as a sequence of consecutive 
 * 2D waypoints(or vertices).
 * 
 */
class Polyline2D
{
    public:
        PointVec2D vertices;

        using Ptr = std::shared_ptr<Polyline2D>;
        using ConstPtr = std::shared_ptr<const Polyline2D>;

        /**
         * @brief Construct a new empty Polyline2D object
         * 
         */
        Polyline2D() = default;

        /**
         * @brief Copy constructor
         * 
         * @param polyline The polyline to be copied
         */
        Polyline2D(const Polyline2D& polyline):
            vertices(polyline.vertices) {}

        /**
         * @brief Construct a new Polyline2D object from a vector of 2D points
         * 
         * @param verts An ordered vector of 2D points representing the polyline
         */
        Polyline2D(const PointVec2D& verts):
            vertices(verts) {}

        /**
         * @brief Destroy the Polyline2D object
         * 
         */
        virtual ~Polyline2D() {}

        /**
         * @brief Returns the length of the complete polyline
         * 
         * @return float The length in meters
         */
        virtual float length() const;

        /**
         * @brief Checks if the 2D polyline and a 2D line segment intersect at
         * atleast one point
         * 
         * @param line_segment The 2D line segment to be checked for intersection
         * @return bool True if the polyline intersects the 2D line segment at
         * atleast one point, false otherwise
         */
        virtual bool intersects(const LineSegment2D& line_segment) const;

        /**
         * @brief Checks if the 2D polyline intersects with another 2D polyline at
         * atleast one point
         * 
         * @param polyline The 2D polyline to be checked for intersection
         * @return bool True if the polyline intersects the queried polyline at
         * atleast one point, false otherwise
         */
        virtual bool intersects(const Polyline2D& polyline) const;

        /**
         * @brief If the 2D polyline intersects with a 2D line segment, this
         * function returns the intersection point that is closest to the start
         * vertex of the line segment
         *
         * @param line_segment The 2D line segment to be checked for intersection
         * @param intersection_pt The intersection point closest to the start
         * vertex of the line segment
         * @return bool True if the polyline intersects the 2D line segment at
         * atleast one point, false otherwise
         */
        virtual bool calcClosestIntersectionPointWith(
                const LineSegment2D& line_segment,
                Point2D& intersection_pt) const;

        /**
         * @brief If the 2D polyline intersects with a 2D polyline, this
         * function returns the intersection point that is closest to the start
         * vertex of the polyline when travelling along the polyline
         *
         * @param polyline The polyline to be checked for intersection
         * @param intersection_pose The intersection pose closest to the start 
         * vertex of the polyline when travelling along the polyline
         * @param segment_id The first segment of the input polyline that 
         * intersects with this polyline
         * @return bool True if the polyline intersects the polyline at
         * atleast one point, false otherwise
         */
        virtual bool calcClosestIntersectionPoseWith(
                const Polyline2D& polyline,
                Pose2D& intersection_pose,
                unsigned int& segment_id) const;

        /**
         * @brief This function splits the polyline into an ordered list of
         * line segments
         *
         * @param max_segment_length The max length of each line segment. \n 
         * If max_segment_length <= 0, then each segment corresponds to a full
         * edge of the polyline
         * @return std::vector<LineSegment2D> A ordered vector of line segments
         */
        std::vector<LineSegment2D> split(float max_segment_length) const;

        /**
         * @brief This function reverses the direction of the polyline
         */
        void reverse();

        /**
         * @brief Get an RViz visualization marker for the polyline object
         * 
         * @param frame The frame in which the polyline marker points are specified
         * @param red The red color-component to be used in the line marker
         * color in the range [0.0, 1.0]
         * @param green The green color-component to be used in the line marker
         * color in the range [0.0, 1.0]
         * @param blue The blue color-component to be used in the line marker
         * color in the range [0.0, 1.0]
         * @param alpha The transparency of the generated line marker
         * in the range [0.0, 1.0]
         * @param line_width The width of the line marker
         * @param use_line_strip whether to use LINE_STRIP or LINE_LIST type
         * @return visualization_msgs::msg::Marker A marker object representing the polyline
         */
        virtual visualization_msgs::msg::Marker asMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float line_width = 0.1f,
                bool use_line_strip = true) const;

        /**
         * @brief Get the number of vertices representing the polyline
         * 
         * @return size_t Num of vertices
         */
        inline size_t size() const
        {
            return vertices.size();
        }

        /**
         * @brief 
         * 
         * @param other 
         * @return Polyline2D& 
         */
        Polyline2D& operator = (const Polyline2D& other);

        /**
         * @brief Indexing operator to access each individual vertex by its index.
         * This method can be used to update the vertex data.
         * 
         * @param index The index of the vertex to be accessed.
         * @return Point2D& The polyline vertex at the specified index
         */
        Point2D& operator [] (unsigned int index);

        /**
         * @brief Indexing operator to access each individual vertex by its index.
         * The returned vertex data cannot be modified
         * 
         * @param index The index of the vertex to be accessed.
         * @return const Point2D& The polyline vertex at the specified index
         */
        const Point2D& operator [] (unsigned int index) const;

        /**
         * @brief Equality checking operator overload. Checks if the members are
         * almost equal by checking if difference is smaller than threshold
         *
         * @param other rhs Polyline2D object
         * @return bool true is all members are almost equal; false otherwise
         */
        bool operator == (const Polyline2D& other) const;

        /**
         * @brief Inequality checking operator overload.
         *
         * @param other rhs Polyline2D object
         * @return bool false is all members are almost equal; true otherwise
         */
        bool operator != (const Polyline2D& other) const;

        /**
         * @brief Append the Polyline information as string to the input stream object
         * 
         * @param out The stream object to which the polyline information should be appended
         * @param polyline The polyline whose data should be appended to the stream object
         * @return std::ostream& The stream object representing the concatenation
         * of the input stream and the polyline information
         */
        friend std::ostream& operator << (
                std::ostream& out,
                const Polyline2D& polyline);
};

} // namespace geometry_common
} // namespace kelo
