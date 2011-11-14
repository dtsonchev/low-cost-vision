#include "huniplacer/deltarobot.h"

#include <sstream>
#include <string>
#include <cstdio>
#include <stdexcept>

#include <huniplacer/motion.h>
#include <huniplacer/inverse_kinematics_model.h>
#include <huniplacer/inverse_kinematics_exception.h>

namespace huniplacer
{
    deltarobot::deltarobot(inverse_kinematics_model& kinematics, imotor3& motors) :
        kinematics(kinematics),
        motors(motors),
        ison(false)
    {
    }

    deltarobot::~deltarobot(void)
    {
        stop();
        motors.moveto(
            motionf(
                0, 0, 0,
                90, 90, 90,
                360, 360, 360,
                360, 360, 360),
            false);
    }
    
    bool deltarobot::is_valid_angle(double angle)
    {
        return angle > motors.get_min_angle() && angle < motors.get_max_angle();
    }

    void deltarobot::moveto(const point3& p, double speed, bool async)
    {
    	if(!ison)
    	    return;

        motionf mf;
        try
        {
        	kinematics.point_to_motion(p, mf);
        }
        catch(inverse_kinematics_exception& ex)
        {
        	throw ex;
        }
        
        if(
            !is_valid_angle(mf.angles[0]) ||
            !is_valid_angle(mf.angles[1]) ||
            !is_valid_angle(mf.angles[2]))
        {
            throw inverse_kinematics_exception("motion angles outside of valid range", p);
        }
        
        //quickfix
        mf.speed[0] = speed;
        mf.speed[1] = speed;
        mf.speed[2] = speed;
        
        try
        {
        	motors.moveto(mf, async);
        }
        catch(std::out_of_range& ex) { throw ex; }
    }
    
    void deltarobot::stop(void)
    {
    	if(!ison)
    	    return;
        motors.stop();
    }
    
    bool deltarobot::wait_for_idle(long timeout)
    {
    	if(!ison)
    	    return false;
        return motors.wait_for_idle(timeout);
    }
    
    bool deltarobot::is_idle(void)
    {
    	if(!ison)
    	    return false;
        return motors.is_idle();
    }

    void huniplacer::deltarobot::power_off(void)
    {
        if(ison)
        {
            motors.power_off();
            ison = false;
        }
    }

    void huniplacer::deltarobot::power_on(void)
    {
        if(!ison)
        {
            motors.power_on();
            ison = true;
        }
    }
}

