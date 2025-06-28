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

#include <vector>
#include <string>
#include <iostream>

namespace kelo
{
namespace geometry_common
{

/**
 * @brief 
 * 
 */
enum class WindingOrder
{
    INVALID = 0,
    CLOCKWISE,
    COUNTER_CLOCKWISE,
    COLLINEAR
};

const std::vector<std::string> winding_order_strings = {
    "INVALID",
    "CLOCKWISE",
    "COUNTER_CLOCKWISE",
    "COLLINEAR",
};

inline std::string asString(const WindingOrder& winding_order)
{
    size_t winding_order_int = static_cast<size_t>(winding_order);
    return ( winding_order_int >= winding_order_strings.size() )
           ? winding_order_strings[0]
           : winding_order_strings[winding_order_int];
};

inline WindingOrder asWindingOrder(const std::string& winding_order_string)
{
    WindingOrder winding_order = WindingOrder::INVALID;
    for ( size_t i = 0; i < winding_order_strings.size(); i++ )
    {
        if ( winding_order_strings[i] == winding_order_string )
        {
            winding_order = static_cast<WindingOrder>(i);
            break;
        }
    }
    return winding_order;
};

} // namespace geometry_common
} // namespace kelo
