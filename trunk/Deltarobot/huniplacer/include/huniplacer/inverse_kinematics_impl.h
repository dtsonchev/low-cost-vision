#pragma once

#include <huniplacer/point3.h>
#include <huniplacer/motion.h>
#include <huniplacer/inverse_kinematics_model.h>

namespace huniplacer
{
	/**
	 * @brief an implementation of kinematics_model
	 **/
	class inverse_kinematics_impl : public inverse_kinematics_model
    {
        private:
            /**
             * @brief translates a point to an angle for a motor
             * @param p destination point
             * @param angle angle of the motor relative to motor 0 (in radians)
             * @return angle the motor should move to
             **/
            double moveto(const point3& p, double angle);
            
        public:
            inverse_kinematics_impl(
				const double base, const double hip, const double effector, const double ankle,
				const double hip_ankle_angle_max);

            virtual ~inverse_kinematics_impl(void);

            /**
             * @brief translates a point to a motion
             * @param p destination point
             * @param mf output parameter, the resulting motion is stored here
             * @return true on success, false otherwise
             **/
            void point_to_motion(const point3& p, motionf& mf);
    };
}
