#pragma once

#include <vector>

#include <huniplacer/motion.h>

namespace huniplacer
{
	/**
	 * @brief interface for the 3 motors
	 **/
	class imotor3
    {
        protected:
            imotor3(void) { }
            
        public:
            virtual ~imotor3(void) { }

            /**
             * @brief rotate the motors
             * @param mf defines the angles, speed, acceleration and deceleration of the motors
             * @param async function is performed asyncronous if true
             **/
            virtual void moveto(const motionf& mf, bool async) = 0;


            /**
             * @brief same as moveto, but rotates to an angle within a certain time.
             * @param time time in seconds that the motors will take to rotate to the given angle
             * speed members of given motion is ignored
             */
            virtual void moveto_within(const motionf& mf, double time, bool async) = 0;

            /**
             * @brief get the minimal angle the motors can move to
             * @return angle in radians
             **/
            virtual double get_min_angle(void) = 0;

            /**
             * @brief get the maximum angle the motors can move to
             * @return angle in radians
             **/
            virtual double get_max_angle(void) = 0;

            /**
             * @brief stop the motors
             **/
            virtual void stop(void) = 0;

            /**
             * @brief wait for all motor controllers to become idle
             * @param timeout time in milliseconds until timeout (0 means infinite)
             * @return false if timed out, true otherwise
             **/
            virtual bool wait_for_idle(long timeout) = 0;

            /**
             * @brief test whether the motor controllers are idle
             * @return true if idle, false otherwise
             **/
            virtual bool is_idle(void) = 0;

            /**
             * @brief shuts down the motors
             */
            virtual void power_off(void) = 0;

            /**
             * @brief turns on the motors
             */
            virtual void power_on(void) = 0;

            /**
             * @brief determine if the motor driver(s) are powered on
             * @return true of powered on, false otherwise
             */
            virtual bool is_powerd_on(void) = 0;
    };
}
