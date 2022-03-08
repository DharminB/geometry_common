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

#include <cmath>
#include <geometry_common/Utils.h>
#include <geometry_common/PointCloudProjector.h>

namespace kelo
{

using geometry_common::Point2D;
using geometry_common::Point3D;
using geometry_common::PointCloud2D;
using geometry_common::PointCloud3D;
using geometry_common::Utils;

void PointCloudProjector::configureTransform(
        float cam_x,
        float cam_y,
        float cam_z,
        float cam_roll,
        float cam_pitch,
        float cam_yaw)
{
    camera_to_target_tf_mat_.update(
            cam_x, cam_y, cam_z, cam_roll, cam_pitch, cam_yaw);
}

bool PointCloudProjector::configure(
        float cam_x,
        float cam_y,
        float cam_z,
        float cam_roll,
        float cam_pitch,
        float cam_yaw,
        float passthrough_min_z,
        float passthrough_max_z,
        float radial_dist_min,
        float radial_dist_max,
        float angle_min,
        float angle_max,
        float angle_increment)
{
    configureTransform(cam_x, cam_y, cam_z, cam_roll, cam_pitch, cam_yaw);

    passthrough_min_z_ = passthrough_min_z;
    passthrough_max_z_ = passthrough_max_z;
    radial_dist_min_ = radial_dist_min;
    radial_dist_min_sq_ = std::pow(radial_dist_min_, 2);
    radial_dist_max_ = radial_dist_max;
    radial_dist_max_sq_ = std::pow(radial_dist_max_sq_, 2);
    angle_min_ = angle_min;
    angle_max_ = angle_max;
    is_angle_flipped_ = ( angle_min_ > angle_max_ );
    angle_increment_ = angle_increment;
    angle_increment_inv_ = 1.0f/angle_increment_;

    num_of_scan_pts_ = PointCloudProjector::calcNumOfScanPts(
            angle_min_, angle_max_, angle_increment_);
    return true;
}

bool PointCloudProjector::configure(
        const YAML::Node& config_params_yaml)
{
    if ( !config_params_yaml.IsMap() )
    {
        std::cout << std::endl << std::endl;
        std::cerr << "Config file does not have a correct format." << std::endl;
        std::cout << std::endl << std::endl;
        return false;
    }

    float x = 0.0f, y = 0.0f, z = 0.0f, roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    if ( config_params_yaml["transform"] )
    {
        const YAML::Node& transform_yaml = config_params_yaml["transform"];
        if ( !transform_yaml.IsMap() )
        {
            std::cout << std::endl << std::endl;
            std::cerr << "Transform in config file does not have a correct format." << std::endl;
            std::cout << std::endl << std::endl;
            return false;
        }

        x     = ( transform_yaml["x"] )     ? transform_yaml["x"].as<float>()     : 0.0f;
        y     = ( transform_yaml["y"] )     ? transform_yaml["y"].as<float>()     : 0.0f;
        z     = ( transform_yaml["z"] )     ? transform_yaml["z"].as<float>()     : 0.0f;
        roll  = ( transform_yaml["roll"] )  ? transform_yaml["roll"].as<float>()  : 0.0f;
        pitch = ( transform_yaml["pitch"] ) ? transform_yaml["pitch"].as<float>() : 0.0f;
        yaw   = ( transform_yaml["yaw"] )   ? transform_yaml["yaw"].as<float>()   : 0.0f;
    }

    float passthrough_min_z = ( config_params_yaml["passthrough_min_z"] )
                              ? config_params_yaml["passthrough_min_z"].as<float>()
                              : passthrough_min_z_;
    float passthrough_max_z = ( config_params_yaml["passthrough_max_z"] )
                              ? config_params_yaml["passthrough_max_z"].as<float>()
                              : passthrough_max_z_;
    float radial_dist_min = ( config_params_yaml["radial_dist_min"] )
                            ? config_params_yaml["radial_dist_min"].as<float>()
                            : radial_dist_min_;
    float radial_dist_max = ( config_params_yaml["radial_dist_max"] )
                            ? config_params_yaml["radial_dist_max"].as<float>()
                            : radial_dist_max_;
    float angle_min = ( config_params_yaml["angle_min"] )
                      ? config_params_yaml["angle_min"].as<float>()
                      : angle_min_;
    float angle_max = ( config_params_yaml["angle_max"] )
                      ? config_params_yaml["angle_max"].as<float>()
                      : angle_max_;
    float angle_increment = ( config_params_yaml["angle_increment"] )
                            ? config_params_yaml["angle_increment"].as<float>()
                            : angle_increment_;

    return configure(x, y, z, roll, pitch, yaw, passthrough_min_z, passthrough_max_z,
                     radial_dist_min, radial_dist_max, angle_min, angle_max, angle_increment);
}

bool PointCloudProjector::configure(
        const std::string& config_file)
{
    YAML::Node config_params_yaml;
    try
    {
        config_params_yaml = YAML::LoadFile(config_file);
    }
    catch(YAML::BadFile&)
    {
        std::cout << std::endl << std::endl;
        std::cerr << "YAML threw BadFile exception. Does the file exist?" << std::endl;
        std::cout << config_file << std::endl;
        std::cout << std::endl << std::endl;
        return false;
    }
    return configure(config_params_yaml);
}

PointCloud3D PointCloudProjector::transformAndFilterPointCloud(
        const PointCloud3D& cloud_in) const
{
    PointCloud3D cloud_out;
    cloud_out.reserve(cloud_in.size());
    for ( const Point3D& pt : cloud_in )
    {
        Point3D transformed_pt = camera_to_target_tf_mat_ * pt;
        if ( isPointValid(transformed_pt) )
        {
            cloud_out.push_back(transformed_pt);
        }
    }
    return cloud_out;
}

std::vector<float> PointCloudProjector::projectedPointCloudToScan(
        const PointCloud3D& cloud_in,
        float angle_min,
        float angle_max) const
{
    size_t num_of_scan_pts = PointCloudProjector::calcNumOfScanPts(
            angle_min, angle_max, angle_increment_);
    bool is_angle_flipped = ( angle_min > angle_max );

    std::vector<float> scan(num_of_scan_pts, radial_dist_max_);

    for ( const Point3D& pt : cloud_in )
    {
        float dist = std::sqrt(std::pow(pt.x, 2) + std::pow(pt.y, 2));
        float angle = std::atan2(pt.y, pt.x);
        if ( is_angle_flipped && angle < angle_max && angle > -M_PI )
        {
            angle += 2*M_PI;
        }
        size_t scan_index = ((angle - angle_min) * angle_increment_inv_) + 0.5f;
        if ( scan_index < num_of_scan_pts )
        {
            scan[scan_index] = std::min(dist, scan[scan_index]);
        }
    }
    return scan;
}

std::vector<float> PointCloudProjector::projectToScan(
        const PointCloud3D& cloud_in,
        float angle_min,
        float angle_max) const
{
    PointCloud3D filtered_cloud;
    return projectToScan(cloud_in, filtered_cloud, angle_min, angle_max);
}

std::vector<float> PointCloudProjector::projectToScan(
        const PointCloud3D& cloud_in,
        PointCloud3D& filtered_cloud,
        float angle_min,
        float angle_max) const
{
    filtered_cloud = transformAndFilterPointCloud(cloud_in);
    return projectedPointCloudToScan(filtered_cloud, angle_min, angle_max);
}

std::vector<float> PointCloudProjector::projectToScan(
        const PointCloud3D& cloud_in) const
{
    PointCloud3D filtered_cloud;
    return projectToScan(cloud_in, filtered_cloud);
}

std::vector<float> PointCloudProjector::projectToScan(
        const PointCloud3D& cloud_in,
        PointCloud3D& filtered_cloud) const
{
    filtered_cloud = transformAndFilterPointCloud(cloud_in);
    return projectedPointCloudToScan(filtered_cloud, angle_min_, angle_max_);
}

PointCloud2D PointCloudProjector::projectToPointCloud2D(
        const PointCloud3D& cloud_in) const
{
    PointCloud3D filtered_cloud;
    return projectToPointCloud2D(cloud_in, filtered_cloud);
}

PointCloud2D PointCloudProjector::projectToPointCloud2D(
        const PointCloud3D& cloud_in,
        PointCloud3D& filtered_cloud) const
{
    filtered_cloud = transformAndFilterPointCloud(cloud_in);
    std::vector<float> scan = projectedPointCloudToScan(
            filtered_cloud, angle_min_, angle_max_);

    /* convert from scan to flat pointcloud */
    PointCloud2D flat_cloud;
    flat_cloud.reserve(scan.size());
    for ( size_t i = 0; i < scan.size(); i++ )
    {
        if ( scan[i] == radial_dist_max_ )
        {
            continue;
        }
        flat_cloud.push_back(Point2D(scan[i] * std::cos(angle_min_ + i*angle_increment_),
                                     scan[i] * std::sin(angle_min_ + i*angle_increment_)));
    }
    return flat_cloud;
}

bool PointCloudProjector::isPointValid(
        const Point3D& pt) const
{
    float angle = std::atan2(pt.y, pt.x);
    float dist_sq = std::pow(pt.x, 2) + std::pow(pt.y, 2);
    if ( std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z) )
    {
        return false;
    }

    /* passthrough z filter */
    if ( pt.z < passthrough_min_z_ || pt.z > passthrough_max_z_ )
    {
        return false;
    }

    /* radial dist filter */
    if ( dist_sq < radial_dist_min_sq_ || dist_sq > radial_dist_max_sq_ )
    {
        return false;
    }

    /* radial angle filter */
    if ( is_angle_flipped_ && ( angle < angle_min_ && angle > angle_max_ ) )
    {
        return false;
    }
    if ( !is_angle_flipped_ && ( angle < angle_min_ || angle > angle_max_ ) )
    {
        return false;
    }

    /* check external validity function */
    if ( external_validity_func_ != nullptr && !external_validity_func_(pt) )
    {
        return false;
    }

    return true;
}

float PointCloudProjector::getRadialDistMax() const
{
    return radial_dist_max_;
}

float PointCloudProjector::getRadialDistMin() const
{
    return radial_dist_min_;
}

void PointCloudProjector::setValidityFunction(
        ValidityFunction vf)
{
    external_validity_func_ = vf;
}

void PointCloudProjector::setPassthroughMinZ(
        float passthrough_min_z)
{
    passthrough_min_z_ = passthrough_min_z;
}

void PointCloudProjector::setPassthroughMaxZ(
        float passthrough_max_z)
{
    passthrough_max_z_ = passthrough_max_z;
}

size_t PointCloudProjector::calcNumOfScanPts(
        float angle_min,
        float angle_max,
        float angle_increment)
{
    return ( angle_min > angle_max ) // angle is flipped for looking backwards
           ? 1 + std::round((angle_max - angle_min + 2*M_PI) / angle_increment)
           : 1 + std::round((angle_max - angle_min) / angle_increment);
}

} // namespace kelo
