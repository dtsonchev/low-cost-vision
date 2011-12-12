#include <sstream>
#include <string>
#include <cstdio>
#include <stdexcept>
#include <cmath>

#include <huniplacer/point3.h>
#include <huniplacer/imotor3.h>
#include <huniplacer/motor3_exception.h>
#include <huniplacer/effector_boundaries.h>
#include <huniplacer/inverse_kinematics_exception.h>
#include <huniplacer/deltarobot.h>

namespace huniplacer
{
    deltarobot::deltarobot(inverse_kinematics_model& kinematics, imotor3& motors) :
        kinematics(kinematics),
        motors(motors),
        effector_location(point3(0, 0, -205)),
        boundaries_generated(false)
    {
    }

    deltarobot::~deltarobot(void)
    {
    	if(motors.is_powerd_on())
    	{
    		motors.stop();
    	}
    }
    
    void deltarobot::generate_boundaries(double voxel_size){
    	boundaries = effector_boundaries::generate_effector_boundaries(kinematics, motors, voxel_size);
    	boundaries_generated = true;
    }

    bool deltarobot::is_valid_angle(double angle)
    {
        return angle > motors.get_min_angle() && angle < motors.get_max_angle();
    }

    void deltarobot::moveto(const point3& p, double speed, bool async)
    {
    	if(!motors.is_powerd_on())
    	{
    		throw motor3_exception("motor drivers are not powered on");
    	}

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
        

    	if(!boundaries->check_path(effector_location, p))
    	{
    		throw inverse_kinematics_exception("invalid path", p);
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
    	if(!motors.is_powerd_on())
		{
			throw motor3_exception("motor drivers are not powered on");
		}
        motors.stop();
    }
    
    bool deltarobot::wait_for_idle(long timeout)
    {
    	if(motors.is_powerd_on())
    	{
    		return motors.wait_for_idle(timeout);
    	}
    	return true;
    }
    
    bool deltarobot::is_idle(void)
    {
        return motors.is_idle();
    }

    void huniplacer::deltarobot::power_off(void)
    {
        if(motors.is_powerd_on())
        {
        	motors.power_off();
        }
    }

    void huniplacer::deltarobot::power_on(void)
    {
        if(!motors.is_powerd_on())
        {
        	motors.power_on();
        }
    }
}
