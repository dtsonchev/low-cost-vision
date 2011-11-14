#pragma once

#include <stdexcept>

#include <huniplacer/motion.h>
#include <huniplacer/point3.h>

namespace huniplacer
{
	/**
	 * @brief instances of deltarobot will throw this exception whenever they fail to convert a point
	 **/
    class inverse_kinematics_exception : public std::runtime_error
    {
        private:
            point3 p;
            
        public:
            inverse_kinematics_exception(const char* msg, point3 p) :
                std::runtime_error(msg),
                p(p)
            {
            }
            
            virtual ~inverse_kinematics_exception(void) throw()
            {
            }

            /**
             * @brief used to access the point that could not be converted
             **/
            point3 get_point(void)
            {
                return p;
            }
    };
}
