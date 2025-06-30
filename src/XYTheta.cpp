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

#include "geometry_common/Utils.h"
#include "geometry_common/XYTheta.h"

namespace kelo
{
namespace geometry_common
{

XYTheta& XYTheta::operator = (const XYTheta& other)
{
    x = other.x;
    y = other.y;
    theta = other.theta;
    return *this;
}

XYTheta XYTheta::operator + (const XYTheta& other) const
{
    XYTheta sum;
    sum.x = x + other.x;
    sum.y = y + other.y;
    sum.theta = theta + other.theta;
    return sum;
}

XYTheta XYTheta::operator - (const XYTheta& other) const
{
    XYTheta diff;
    diff.x = x - other.x;
    diff.y = y - other.y;
    diff.theta = theta - other.theta;
    return diff;
}

XYTheta XYTheta::operator * (float scalar) const
{
    XYTheta scaled;
    scaled.x = x * scalar;
    scaled.y = y * scalar;
    scaled.theta = theta * scalar;
    return scaled;
}

XYTheta XYTheta::operator / (float scalar) const
{
    if ( std::fabs(scalar) < 1e-9f ) // to fix divide by zero issue
    {
        scalar = 1e-9f;
    }
    return (*this) * (1.0f/scalar);
}

bool XYTheta::operator == (const XYTheta& other) const
{
    XYTheta diff = *this - other;
    return ( pow(diff.x, 2) + pow(diff.y, 2) + pow(diff.theta, 2) < 1e-6f );
}

bool XYTheta::operator != (const XYTheta& other) const
{
    return !((*this) == other);
}

std::ostream& operator << (std::ostream& out, const XYTheta& x_y_theta)
{
    out <<  "<x: " << x_y_theta.x
        << ", y: " << x_y_theta.y
        << ", theta: " << x_y_theta.theta
        << ">";
    return out;
}

} // namespace geometry_common
} // namespace kelo
