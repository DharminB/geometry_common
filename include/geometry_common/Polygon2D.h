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

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "geometry_common/Polyline2D.h"

namespace kelo
{
namespace geometry_common
{

/**
 * @brief Represents a polygon as a sequence of consecutive 2D waypoints
 * (or vertices). A N-sided polygon is represented by N vertices.An implicit
 * assumption is that there exists a polygon edge from the last to first node
 * in the sequence. \n \n
 * For example, a triangle with vertices [**a**, **b**, **c**] will have the 
 * following edges:
 * 1. **a** -> **b**
 * 2. **b** -> **c**
 * 3. **c** -> **a**
 * 
 */
class Polygon2D : public Polyline2D
{
    public:
        using Ptr = std::shared_ptr<Polygon2D>;
        using ConstPtr = std::shared_ptr<const Polygon2D>;

        /**
         * @brief Construct a new empty Polygon2D object
         * 
         */
        Polygon2D():
            Polyline2D() {}

        /**
         * @brief Copy constructor
         * 
         * @param polygon The polygon to be copied
         */
        Polygon2D(const Polygon2D& polygon):
            Polyline2D(polygon) {}

        /**
         * @brief Construct a new Polygon2D object from polyline
         * 
         * @param polyline A polyline representing the boundary of the polygon.
         * The line must not have the first vertex repeated at the end of the polyline
         */
        Polygon2D(const Polyline2D& polyline):
            Polyline2D(polyline) {}

        /**
         * @brief Construct a new Polygon2D object from a vector of 2D points
         * 
         * @param verts An ordered vector of 2D points representing the vertices
         * of the polygon
         */
        Polygon2D(const PointVec2D& verts):
            Polyline2D(verts) {}

        /**
         * @brief Destroy the Polygon2D object
         * 
         */
        virtual ~Polygon2D() {}

        /**
         * @brief Returns the perimeter of the polygon
         * 
         * @return float The perimeter in meters
         */
        float length() const override;

        /**
         * @brief Checks if the 2D polygon and a 2D line segment intersect at
         * atleast one point
         * 
         * @param line_segment The 2D line segment to be checked for intersection
         * @return bool True if the polygon intersects the 2D line segment at
         * atleast one point, false otherwise
         */
        bool intersects(const LineSegment2D& line_segment) const override;

        /**
         * @brief Checks if the 2D polygon intersects with 2D polyline at
         * atleast one point
         * 
         * @param polyline The 2D polyline to be checked for intersection
         * @return bool True if the polygon intersects the queried polyline at
         * atleast one point, false otherwise
         */
        bool intersects(const Polyline2D& polyline) const override;

        /**
         * @brief If the 2D polygon intersects with a 2D line segment, this function returns the 
         * intersection point that is closest to the start vertex of the line segment
         * @param line_segment The 2D line segment to be checked for intersection
         * @param intersection_pt The intersection point closest to the start vertex of the line segment
         * @return bool True if the polygon intersects the 2D line segment at
         * atleast one point, false otherwise
         */
        bool calcClosestIntersectionPointWith(
                const LineSegment2D& line_segment,
                Point2D& intersection_pt) const override;

        /**
         * @brief If the 2D polygon intersects with a 2D polyline, this
         * function returns the intersection point that is closest to the start
         * vertex of the polyline when travelling along the polyline
         *
         * @param polyline The polyline to be checked for intersection
         * @param intersection_pose The intersection pose closest to the start 
         * vertex of the polyline when travelling along the polyline
         * @param segment_id The first segment of the input polyline that 
         * intersects with this polyline
         * @return bool True if the polygon intersects the polyline at
         * atleast one point, false otherwise
         */
        bool calcClosestIntersectionPoseWith(
                const Polyline2D& polyline,
                Pose2D& intersection_pose,
                unsigned int& segment_id) const;

        /**
         * @brief Check if a 2D point lies within the polygon.
         * 
         * @param point The 2D point to be checked
         * @return bool True if the point lies inside the polygon, false otherwise
         */
        bool containsPoint(const Point2D& point) const;

        /**
         * @brief Check if atleast one of the point from the input points vector
         * lies within the polygon
         * 
         * @param points A vector of points to be checked
         * @return bool True if even one of the points lies inside the polygon,
         * false otherwise
         */
        bool containsAnyPoint(const PointVec2D& points) const;

        /**
         * @brief Get the mean of all the polygon vertices
         * 
         * @return Point2D The computed mean point
         */
        Point2D meanPoint() const;

        /**
         * @brief Get the area of the polygon
         * 
         * @note If the sign of the area is negative, it means the polygon
         * vertices are specified in a clock-wise winding order
         * 
         * @return float The area of the polygon
         */
        float area() const;

        /**
         * @brief Check if the polygon is convex
         * 
         * @return bool True if polygon is convex, false otherwise
         */
        bool isConvex() const;

        /**
         * @brief Check if the polygon is convex within certain tolerance.
         * Also ignores any repeated consecutive points in the polygon
         * 
         * @param tolerance The tolerance in radians to ignore minor angular 
         * fluctuations along a straight polygon edge composed of multiple line
         * segments. (Default: 1 degree)
         * @return bool True if polygon is convex, false otherwise
         */
        bool isApproximatelyConvex(float tolerance = 0.017453293) const;

        /**
         * @brief Find convex hull from the union of 2 polygons.
         * 
         * @param polygon_a First polygon that will be part of the union
         * @param polygon_b Second polygon that will be part of the union
         * @return Polygon2D A polygon representing the convex hull of the 
         * unified input polygons
         */
        static Polygon2D calcConvexHullOfPolygons(
                const Polygon2D& polygon_a,
                const Polygon2D& polygon_b);

        /**
         * @brief Inflate/Deflate a polygon by given distance.
         * 
         * @param inflation_dist Distance by which to inflate/deflate the
         * polygon. Positive number would inflate and negative number would
         * deflate.
         * @return Polygon2D An inflated/deflated polygon
         */
        Polygon2D calcInflatedPolygon(float inflation_dist) const;

        /**
         * @brief Get an RViz visualization marker for the polygon object.
         * The generated marker object will be a non-filled polygon boundary.
         * 
         * @param frame The frame in which the polygon marker points are specified
         * @param red The red color-component to be used in the line marker
         * color in the range [0.0, 1.0]
         * @param green The green color-component to be used in the line marker
         * color in the range [0.0, 1.0]
         * @param blue The blue color-component to be used in the line marker
         * color in the range [0.0, 1.0]
         * @param alpha The transparency of the generated line marker
         * in the range [0.0, 1.0]
         * @param line_width The width of the line marker
         * @return visualization_msgs::msg::Marker A marker object representing the polygon
         */
        visualization_msgs::msg::Marker asMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float line_width = 0.1f,
                bool use_line_strip = true) const override;

        /**
         * @brief Get a PolygonStamped message object from Polygon2D object
         *
         * @param frame The frame in which the polygon stamped message needs to
         * be published
         * @return geometry_msgs::msg::PolygonStamped message object
         */
        geometry_msgs::msg::PolygonStamped asPolygonStamped(
                const std::string& frame = "base_link") const;

        /**
         * @brief 
         * 
         * @param other 
         * @return Polygon2D& 
         */
        Polygon2D& operator = (const Polygon2D& other);

        /**
         * @brief Append the polygon information as string to the input stream object
         * 
         * @param out The stream object to which the polygon information should be appended
         * @param polygon The polygon whose data should be appended to the stream object
         * @return std::ostream& The stream object representing the concatenation
         * of the input stream and the polygon information
         */
        friend std::ostream& operator << (
                std::ostream& out,
                const Polygon2D& polygon);
};

} // namespace geometry_common
} // namespace kelo
