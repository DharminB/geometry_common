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

#include <cmath>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"

#include "geometry_common/Point2D.h"
#include "geometry_common/XYTheta.h"

namespace kelo
{
namespace geometry_common
{

// Forward declaration 
class TransformMatrix2D;

/**
 * @brief Pose in two dimensional space.
 * 
 */
class Pose2D : public XYTheta
{
    public:
        using Ptr = std::shared_ptr<Pose2D>;
        using ConstPtr = std::shared_ptr<const Pose2D>;

        /**
         * @brief
         * 
         * @param _x 
         * @param _y 
         * @param _theta 
         */
        Pose2D(float _x = 0.0f, float _y = 0.0f, float _theta = 0.0f);

        /**
         * @brief
         * 
         * @param pose 
         */
        Pose2D(const Pose2D& pose):
            Pose2D(pose.x, pose.y, pose.theta) {}

        /**
         * @brief 
         *
         * @param x_y_theta
         */
        Pose2D(const XYTheta& x_y_theta):
            Pose2D(x_y_theta.x, x_y_theta.y, x_y_theta.theta) {}

        /**
         * @brief
         * 
         * @param position 
         * @param theta 
         */
        Pose2D(const Point2D& position, float theta = 0.0f):
            Pose2D(position.x, position.y, theta) {}

        /**
         * @brief
         * 
         * @param pose 
         */
        Pose2D(const geometry_msgs::msg::PoseStamped& pose);

        /**
         * @brief
         * 
         * @param pose 
         */
        Pose2D(const geometry_msgs::msg::Pose& pose);

        /**
         * @brief
         * 
         * @param mat 
         */
        Pose2D(const TransformMatrix2D& mat);

        /**
         * @brief
         * 
         * @param ts transform stamped message
         */
        Pose2D(const geometry_msgs::msg::TransformStamped& ts);

        /**
         * @brief
         * 
         */
        virtual ~Pose2D() {}

        /**
         * @brief 
         * 
         * @return Point2D 
         */
        inline Point2D position() const { return Point2D(x, y); }

        /**
         * @brief Update x and y attribute of pose from input point
         *
         * @param pt Point whose x and y will be used for update
         */
        void updatePosition(const Point2D& pt);

        /**
         * @brief
         * 
         * @param frame 
         * @return geometry_msgs::msg::PoseStamped 
         */
        geometry_msgs::msg::PoseStamped asPoseStamped(
                const std::string& frame = "map") const;

        /**
         * @brief
         * 
         * @return geometry_msgs::msg::Pose 
         */
        geometry_msgs::msg::Pose asPose() const;

        /**
         * @brief
         * 
         * @return TransformMatrix2D
         */
        TransformMatrix2D asMat() const;

        /**
         * @brief
         * 
         * @param frame 
         * @param red 
         * @param green 
         * @param blue 
         * @param alpha 
         * @param size_x 
         * @param size_y 
         * @param size_z 
         * @return visualization_msgs::msg::Marker 
         */
        visualization_msgs::msg::Marker asMarker(
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float size_x = 0.3f,
                float size_y = 0.05f,
                float size_z = 0.05f) const;

        visualization_msgs::msg::InteractiveMarker asInteractiveMarker(
                const std::string& name,
                const std::string& frame = "base_link",
                float red = 1.0f,
                float green = 0.0f,
                float blue = 0.0f,
                float alpha = 1.0f,
                float size_x = 0.3f,
                float size_y = 0.05f,
                float size_z = 0.05f) const;

        /**
         * @brief
         * 
         * @param p 
         * @return float 
         */
        inline float distTo(const Pose2D& p) const
        {
            return std::sqrt(squaredDistTo(p));
        };

        /**
         * @brief
         * 
         * @param p 
         * @return float 
         */
        inline float squaredDistTo(const Pose2D& p) const
        {
            return std::pow(x - p.x, 2) + std::pow(y - p.y, 2);
        };

        /**
         * @brief
         * 
         * @param p 
         * @return float 
         */
        inline float distTo(const Point2D& p) const
        {
            return std::sqrt(std::pow(x - p.x, 2) + std::pow(y - p.y, 2));
        };

        /**
         * @brief 
         * 
         * @return std::string 
         */
        std::string asString() const;

        /**
         * @brief 
         * 
         * @param other 
         * @return Pose2D& 
         */
        Pose2D& operator = (const Pose2D& other);

        /**
         * @brief 
         * 
         * @param other 
         * @return Pose2D 
         */
        Pose2D operator - (const Pose2D& other) const;

        /**
         * @brief Equality checking operator overload. Checks if the members are
         * almost equal by checking if difference is smaller than threshold
         *
         * @param other rhs Pose2D object
         * @return bool true is all members are almost equal; false otherwise
         */
        bool operator == (const Pose2D& other) const;

        /**
         * @brief Inequality checking operator overload.
         *
         * @param other rhs Pose2D object
         * @return bool false is all members are almost equal; true otherwise
         */
        bool operator != (const Pose2D& other) const;

};

/**
 * @brief Path is a ordered collection of poses
 */
using Path = std::vector<Pose2D>;

using PoseVec2D = std::vector<Pose2D>;

} // namespace geometry_common
} // namespace kelo
