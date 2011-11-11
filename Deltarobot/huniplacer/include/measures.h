#pragma once

namespace huniplacer
{
	namespace measures
	{
		const double
			BASE 				= 101.3,  //mm
			HIP 				= 61.1,   //mm
			EFFECTOR 			= 42.15,  //mm
			ANKLE				= 150.8,  //mm
			HIP_ANKLE_ANGLE_MAX = 26.5,   //degrees
			MOTOR_ROT_MIN 		= -45,	  //degrees
			MOTOR_ROT_MAX 		= 75;     //degrees

		const double MAX_X = measures::HIP + measures::ANKLE + measures::EFFECTOR - measures::BASE;
		const double MAX_Y = MAX_X,
					 MIN_X = -MAX_X,
					 MIN_Y = -MAX_Y;
		const double MIN_Z = -150,
				     MAX_Z = 0;
	}
}
