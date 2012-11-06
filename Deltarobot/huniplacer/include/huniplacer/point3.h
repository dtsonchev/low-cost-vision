//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        huniplacer
// File:           point3.h
// Description:    3 dimensional point class
// Author:         Lukas Vermond & Kasper van Nieuwland
// Notes:          -
//
// License: newBSD 
//  
// Copyright © 2012, HU University of Applied Sciences Utrecht. 
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
// - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//******************************************************************************


#pragma once

#include <cmath>

namespace huniplacer
{
	/// @brief 3 dimensional point class
    class point3
    {
        public:
            double x, y, z;

            point3(double x, double y, double z) : x(x), y(y), z(z) { }
            ~point3() { }
            
            point3 offset(point3& p);
            
            inline point3& operator+=(const point3& rhs)
            {
                x += rhs.x;
                y += rhs.y;
                z += rhs.z;
                return *this;
            }
            
            inline const point3 operator+(const point3& rhs) const
            {
                point3 res = *this;
                res += rhs;
                return res;
            }
            
            /**
             * @brief calculates the euclidean distance between *this and p
             * @return distance
             **/
            inline double distance(const point3& p) const
            {
                double dx = x - p.x;
                double dy = y - p.y;
                double dz = z - p.z;
                return sqrt(dx*dx + dy*dy + dz*dz);
            }
            
            /**
             * @brief rotate over the Y axis
             * @param phi rotation in radians
             **/
            inline point3 rotate_y(double phi) const
            {
                return point3(x*cos(phi) - z*sin(phi), y, x*sin(phi) + z*cos(phi));
            }
            
            /**
             * @brief rotate over the Z axis
             * @param phi rotation in radians
             **/
            inline point3 rotate_z(double phi) const
            {
                return point3(x*cos(phi) - y*sin(phi), x*sin(phi) + y*cos(phi), z);
            }            
    };
}
