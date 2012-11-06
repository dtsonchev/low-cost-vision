//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        inverse_kinematics_impl.cpp
// File:           inverse kinematics implementation. based on work from Viacheslav Slavinsky
// Description:    Lukas Vermond & Kasper van Nieuwland
// Author:         -
// Notes:          
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


#include <huniplacer/inverse_kinematics_impl.h>

#include <cmath>
#include <cstdio>
#include <boost/math/special_functions/fpclassify.hpp>
#include <huniplacer/inverse_kinematics_exception.h>

#include <huniplacer/utils.h>

namespace huniplacer
{
    inverse_kinematics_impl::inverse_kinematics_impl(const double base, const double hip, const double effector, const double ankle, const double hip_ankle_angle_max) :
        inverse_kinematics_model(base, hip, effector, ankle, hip_ankle_angle_max)
    {
    }

    inverse_kinematics_impl::~inverse_kinematics_impl(void)
    {
    }

	#define SQR(x) ((x)*(x))
    double inverse_kinematics_impl::moveto(const point3& p, double motor_angle) const
    {
    	//ideas from Viacheslav Slavinsky are used
    	//conventions:
    	//	sitting in front of delta robot
    	//	x-axis goes from left to right
    	//	y-axis goes from front to back
    	//	z-axis goes from bottom to top
    	//	point (0,0,0) lies in the middle of all the motors at the motor's height

    	point3 p_fixed = p.rotate_z(-motor_angle);

    	p_fixed.y -= effector;
    	p_fixed.y += base;

    	//double c = sqrt(SQR(p_fixed.x) + SQR(p_fixed.y) + SQR(p_fixed.z));
    	double c = sqrt(SQR(p_fixed.y) + SQR(p_fixed.z));

    	if(c == 0)
    	{
    		throw inverse_kinematics_exception("point out of range", p);
    	}

    	double alpha_acos_input =
    			(-(SQR(ankle) - SQR(p_fixed.x)) + SQR(hip) + SQR(c))
    			/
    			(2*hip*c);
    	if(alpha_acos_input < -1 || alpha_acos_input > 1)
    	{
    		throw inverse_kinematics_exception("point out of range", p);
    	}

    	double alpha = acos(alpha_acos_input);

    	double beta = atan2(p_fixed.z, p_fixed.y);
    	double rho = beta - alpha;

    	double hip_ankle_angle = asin(abs(p_fixed.x)/ankle);

    	if(hip_ankle_angle > hip_ankle_angle_max)
    	{
    		throw inverse_kinematics_exception("angle between hip and ankle is out of range", p);
    	}

    	return rho;
    }
	#undef SQR
    
    void inverse_kinematics_impl::point_to_motion(const point3& p, motionf& mf) const
    {
        point3 goal = p;

		mf.angles[0] = utils::rad(-90) - moveto(p, utils::rad(0 * 120));
		mf.angles[1] = utils::rad(-90) - moveto(p, utils::rad(1 * 120));
		mf.angles[2] = utils::rad(-90) - moveto(p, utils::rad(2 * 120));

        mf.acceleration[0] = mf.acceleration[1] = mf.acceleration[2] = utils::rad(3600);
        mf.deceleration[0] = mf.deceleration[1] = mf.deceleration[2] = utils::rad(3600);
    }
}

