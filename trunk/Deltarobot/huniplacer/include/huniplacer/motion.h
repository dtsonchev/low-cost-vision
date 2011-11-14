#pragma once

#include <stdint.h>

namespace huniplacer
{
	/**
	 * @brief template class for use in types motioni and motionf (see below)
	 **/
    template<typename T>
    class motion
    {
        public:
            T angles[3];
            T speed[3];
            T acceleration[3];
            T deceleration[3];
            
            motion(void) { }
            
            motion(
                T angle0, T angle1, T angle2,
                T speed0, T speed1, T speed2,
                T acc0, T acc1, T acc2,
                T dec0, T dec1, T dec2)
            {
                angles[0] = angle0;
                angles[1] = angle1;
                angles[2] = angle2;
                
                speed[0] = speed0;
                speed[1] = speed1;
                speed[2] = speed2;
                
                acceleration[0] = acc0;
                acceleration[1] = acc1;
                acceleration[2] = acc2;
                
                deceleration[0] = dec0;
                deceleration[1] = dec1;
                deceleration[2] = dec2;
            }
            
            ~motion(void) { }
    };
    
    /// @brief floating point motion type
    typedef motion<double> motionf;

    /// @brief integer motion type
    typedef motion<uint32_t> motioni;
}
