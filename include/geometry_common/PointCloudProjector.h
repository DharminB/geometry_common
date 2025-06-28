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

#include <iostream>
#include <string>
#include <functional>

#include "geometry_common/Point2D.h"
#include "geometry_common/Point3D.h"
#include "geometry_common/TransformMatrix3D.h"

namespace kelo
{

/**
 * @brief Structure containing all the configuration parameter required to
 * configure PointCloudProjector
 */
struct PointCloudProjectorConfig
{
    geometry_common::TransformMatrix3D tf_mat;
    float passthrough_min_z{0.0f};
    float passthrough_max_z{2.0f};
    float radial_dist_min{0.1f};
    float radial_dist_max{1e3f};
    float angle_min{-3.14f};
    float angle_max{3.14f};
    float angle_increment{0.01f};
};

/**
 * @brief Function pointer type definition for checking validity of a point for
 * filtering.
 * 
 */
typedef std::function<bool (const geometry_common::Point3D& )> ValidityFunction;

/**
 * @brief 
 * 
 */
class PointCloudProjector
{
    public:
        using Ptr = std::shared_ptr<PointCloudProjector>;
        using ConstPtr = std::shared_ptr<const PointCloudProjector>;

        /**
         * @brief
         * 
         */
        PointCloudProjector() = default;

        /**
         * @brief
         * 
         */
        virtual ~PointCloudProjector() {}

        /**
         * @brief 
         * 
         * @param cam_x 
         * @param cam_y 
         * @param cam_z 
         * @param cam_roll 
         * @param cam_pitch 
         * @param cam_yaw 
         */
        void configureTransform(
                float cam_x = 0.0f,
                float cam_y = 0.0f,
                float cam_z = 0.0f,
                float cam_roll = 0.0f,
                float cam_pitch = 0.0f,
                float cam_yaw = 0.0f);

        /**
         * @brief Configure PointCloudProjector
         *
         * @note: This should be called before calling any `projectTo...`
         * functions.
         *
         * @param config All the configuration parameter wrapped in a struct
         * @return success
         */
        bool configure(
                const PointCloudProjectorConfig& config);

        /**
         * @brief 
         * 
         * @param cloud_in 
         * @param angle_min 
         * @param angle_max 
         * @return std::vector<float> 
         */
        std::vector<float> projectToScan(
                const geometry_common::PointCloud3D& cloud_in,
                float angle_min,
                float angle_max) const;

        /**
         * @brief 
         * 
         * @param cloud_in 
         * @param filtered_cloud 
         * @param angle_min 
         * @param angle_max 
         * @return std::vector<float> 
         */
        std::vector<float> projectToScan(
                const geometry_common::PointCloud3D& cloud_in,
                geometry_common::PointCloud3D& filtered_cloud,
                float angle_min,
                float angle_max) const;

        /**
         * @brief 
         * 
         * @param cloud_in 
         * @return std::vector<float> 
         */
        std::vector<float> projectToScan(
                const geometry_common::PointCloud3D& cloud_in) const;

        /**
         * @brief 
         * 
         * @param cloud_in 
         * @param filtered_cloud 
         * @return std::vector<float> 
         */
        std::vector<float> projectToScan(
                const geometry_common::PointCloud3D& cloud_in,
                geometry_common::PointCloud3D& filtered_cloud) const;

        /**
         * @brief 
         * 
         * @param cloud_in 
         * @return geometry_common::PointCloud2D 
         */
        geometry_common::PointCloud2D projectToPointCloud2D(
                const geometry_common::PointCloud3D& cloud_in) const;

        /**
         * @brief 
         * 
         * @param cloud_in 
         * @param filtered_cloud 
         * @return geometry_common::PointCloud2D 
         */
        geometry_common::PointCloud2D projectToPointCloud2D(
                const geometry_common::PointCloud3D& cloud_in,
                geometry_common::PointCloud3D& filtered_cloud) const;

        /**
         * @brief
         * 
         * @return float 
         */
        float getRadialDistMax() const;

        /**
         * @brief
         * 
         * @return float 
         */
        float getRadialDistMin() const;

        /**
         * @brief
         * 
         * @param vf 
         */
        void setValidityFunction(
                ValidityFunction vf);

        /**
         * @brief
         * 
         * @param passthrough_min_z 
         */
        void setPassthroughMinZ(
                float passthrough_min_z);

        /**
         * @brief
         * 
         * @param passthrough_max_z 
         */
        void setPassthroughMaxZ(
                float passthrough_max_z);

        /**
         * @brief set transformation matrix from camera to target frame
         * @param tf_mat
         */
        void setTransform(
                const geometry_common::TransformMatrix3D& tf_mat);

        /**
         * @brief 
         * 
         * @param angle_min 
         * @param angle_max 
         * @param angle_increment 
         * @return size_t 
         */
        static size_t calcNumOfScanPts(
                float angle_min,
                float angle_max,
                float angle_increment);

    protected:
        geometry_common::TransformMatrix3D camera_to_target_tf_mat_;
        float passthrough_min_z_{0.0f};
        float passthrough_max_z_{2.0f};
        float radial_dist_min_{0.1f};
        float radial_dist_max_{1e3f};
        float radial_dist_min_sq_{0.01f};
        float radial_dist_max_sq_{1e6f};
        float angle_min_{-M_PI};
        float angle_max_{M_PI};
        bool is_angle_flipped_{false};
        float angle_increment_{0.01f};
        float angle_increment_inv_{100.0f};
        size_t num_of_scan_pts_{0};

        ValidityFunction external_validity_func_{nullptr};

        /**
         * @brief 
         * 
         * @param pt 
         * @return bool 
         */
        bool isPointValid(
                const geometry_common::Point3D& pt) const;

        /**
         * @brief 
         * 
         * @param cloud_in 
         * @param angle_min 
         * @param angle_max 
         * @return std::vector<float> 
         */
        std::vector<float> projectedPointCloudToScan(
                const geometry_common::PointCloud3D& cloud_in,
                float angle_min,
                float angle_max) const;

        /**
         * @brief 
         * 
         * @param cloud_in 
         * @return geometry_common::PointCloud3D 
         */
        geometry_common::PointCloud3D transformAndFilterPointCloud(
                const geometry_common::PointCloud3D& cloud_in) const;


};

} // namespace kelo
