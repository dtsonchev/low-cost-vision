#pragma once

#include <huniplacer/point3.h>
#include <huniplacer/motion.h>

namespace huniplacer
{
	/**
	 * @brief kinematics model of the deltarobot
	 *
	 * uses various lengths and sizes to calculate a point to a motion
	 *
	 *  conventions:
     *	sitting in front of delta robot
     *	x-axis goes from left to right
     *	y-axis goes from front to back
     *	z-axis goes from bottom to top
     *	point (0,0,0) lies in the middle of all the motors at the motor's height
	 **/
    class inverse_kinematics_model
    {
        protected:
    		/// @brief radius of the base in millimeters
            const double base;

            /// @brief length of the hip in millimeters
            const double hip;

            /// @brief radius of the effector in millimeters
            const double effector;

            /// @brief length of the ankle in millimeters
            const double ankle;

            /// @brief maximum angle between hip and angle on x-z plane
            const double hip_ankle_angle_max;
        
            inverse_kinematics_model(
				const double base, const double hip, const double effector, const double ankle,
				const double hip_ankle_angle_max) :
                base(base),
                hip(hip),
                effector(effector),
                ankle(ankle),
                hip_ankle_angle_max(hip_ankle_angle_max)
            { }
            
        public:
            virtual ~inverse_kinematics_model(void) { }

            /**
             * @brief converts a point to a motion
             * @param p point that shall be converted
             * @param mf output parameter, the results of the conversion will be stored here
             **/
            virtual void point_to_motion(const point3& p, motionf& mf) = 0;
    };
}
