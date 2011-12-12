#pragma once
#include <huniplacer/utils.h>

namespace huniplacer
{
	namespace measures
	{
		const double BASE 				  = 101.3; //mm
		const double HIP 				  = 61.1;  //mm
		const double EFFECTOR 			  = 46.19; //mm old value: 42.15
		const double ANKLE				  = 150.8; //mm
		const double HIP_ANKLE_ANGLE_MAX  = utils::rad(20);  //radians
		const double MOTOR_ROT_MIN 	      = utils::rad(-45);   //radians
		const double MOTOR_ROT_MAX 	      = utils::rad(75);    //radians

		//make sure all the points that the effector can reach are included in the box
		// with the following dimensions:
		const double MAX_X = 85;
		const double MAX_Y = MAX_X;
		const double MIN_X = -MAX_X;
		const double MIN_Y = -MAX_Y;
		const double MIN_Z = -210;
		const double MAX_Z = -100;
	}
}
