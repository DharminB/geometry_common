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

#include "geometry_common/Box3D.h"

namespace kelo
{
namespace geometry_common
{

Box3D::Box3D(const PointVec3D& points)
{
    min_x = std::numeric_limits<float>::max();
    max_x = std::numeric_limits<float>::lowest();
    min_y = std::numeric_limits<float>::max();
    max_y = std::numeric_limits<float>::lowest();
    min_z = std::numeric_limits<float>::max();
    max_z = std::numeric_limits<float>::lowest();

    for ( const Point3D& pt : points )
    {
        if ( pt.x < min_x )
        {
            min_x = pt.x;
        }
        if ( pt.x > max_x )
        {
            max_x = pt.x;
        }
        if ( pt.y < min_y )
        {
            min_y = pt.y;
        }
        if ( pt.y > max_y )
        {
            max_y = pt.y;
        }
        if ( pt.z < min_z )
        {
            min_z = pt.z;
        }
        if ( pt.z > max_z )
        {
            max_z = pt.z;
        }
    }
}

float Box3D::sizeX() const
{
    return max_x - min_x;
}

float Box3D::sizeY() const
{
    return max_y - min_y;
}

float Box3D::sizeZ() const
{
    return max_z - min_z;
}

Point3D Box3D::center() const
{
    return Point3D((min_x + max_x)/2,
                   (min_y + max_y)/2,
                   (min_z + max_z)/2);
}

visualization_msgs::msg::Marker Box3D::asMarker(
        const std::string& frame,
        float red, float green, float blue, float alpha) const
{
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.header.frame_id = frame;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.scale.x = sizeX();
    marker.scale.y = sizeY();
    marker.scale.z = sizeZ();
    marker.pose.position = center().asPoint();
    marker.pose.orientation.w = 1.0f;
    return marker;
}

bool Box3D::containsPoint(const Point3D& p) const
{
    return ( p.x >= min_x && p.x <= max_x &&
             p.y >= min_y && p.y <= max_y &&
             p.z >= min_z && p.z <= max_z );
}

Box3D& Box3D::operator = (const Box3D& other)
{
    min_x = other.min_x;
    max_x = other.max_x;
    min_y = other.min_y;
    max_y = other.max_y;
    min_z = other.min_z;
    max_z = other.max_z;
    return *this;
}

bool Box3D::operator == (const Box3D& other) const
{
    return ( std::fabs(min_x - other.min_x) < 1e-3f &&
             std::fabs(max_x - other.max_x) < 1e-3f &&
             std::fabs(min_y - other.min_y) < 1e-3f &&
             std::fabs(max_y - other.max_y) < 1e-3f &&
             std::fabs(min_z - other.min_z) < 1e-3f &&
             std::fabs(max_z - other.max_z) < 1e-3f );
}

bool Box3D::operator != (const Box3D& other) const
{
    return !((*this) == other);
}

std::ostream& operator << (std::ostream& out, const Box3D& box)
{
    out <<  "<min_x: " << box.min_x
        << ", max_x: " << box.max_x
        << ", min_y: " << box.min_y
        << ", max_y: " << box.max_y
        << ", min_z: " << box.min_z
        << ", max_z: " << box.max_z
        << ">";
    return out;
}

} // namespace geometry_common
} // namespace kelo
