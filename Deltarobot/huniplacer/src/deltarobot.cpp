#include <huniplacer/deltarobot.h>

#include <sstream>
#include <string>
#include <cstdio>
#include <stdexcept>
#include <cmath>

#include <huniplacer/motion.h>
#include <huniplacer/inverse_kinematics_model.h>
#include <huniplacer/inverse_kinematics_exception.h>
#include <huniplacer/utils.h>

namespace huniplacer
{
    deltarobot::deltarobot(inverse_kinematics_model& kinematics, imotor3& motors) :
        kinematics(kinematics),
        motors(motors),
        ison(false),
        effector_location(point3(0,0,-205))
    {


    }

    deltarobot::~deltarobot(void)
    {
        stop();
        motors.moveto(
            motionf(
                0, 0, 0,
                utils::rad(90), utils::rad(90), utils::rad(90),
                utils::rad(360), utils::rad(360), utils::rad(360),
                utils::rad(360), utils::rad(360), utils::rad(360)),
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
        
        double move_time = p.distance(effector_location) / speed;

        try
        {
        	motors.moveto_within(mf, move_time, async);
        }
        catch(std::out_of_range& ex) { throw ex; }

        effector_location = p;
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

