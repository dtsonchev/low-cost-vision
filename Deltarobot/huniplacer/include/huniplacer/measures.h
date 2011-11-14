#pragma once

namespace huniplacer
{
	namespace measures
	{
		const double BASE 				  = 101.3; //mm
		const double HIP 				  = 61.1;  //mm
		const double EFFECTOR 			  = 42.15; //mm
		const double ANKLE				  = 150.8; //mm
		const double HIP_ANKLE_ANGLE_MAX  = 26.5;  //degrees
		const double MOTOR_ROT_MIN 	      = -45;   //degrees
		const double MOTOR_ROT_MAX 	      = 75;    //degrees

		const double MAX_X = measures::HIP + measures::ANKLE + measures::EFFECTOR - measures::BASE;
		const double MAX_Y = MAX_X;
		const double MIN_X = -MAX_X;
		const double MIN_Y = -MAX_Y;
		const double MIN_Z = -150;
		const double MAX_Z = 0;
	}
}
