#include <huniplacer/steppermotor3.h>

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include <huniplacer/utils.h>
#include <huniplacer/CRD514_KD.h>
#include <huniplacer/crd514_kd_exception.h>

namespace huniplacer
{
    steppermotor3::steppermotor3(modbus_t* context, double min_angle, double max_angle, motion_thread_exception_handler exhandler) :
        imotor3(),
        motion_queue(),
        thread_running(true),
        idle(true),
        idle_mutex(), idle_cond(),
        queue_mutex(),
        modbus_mutex(),
        min_angle(min_angle), max_angle(max_angle),
        modbus(context),
        exhandler(exhandler),
        on(false)
    {
    	current_angles[0] = current_angles[1] = current_angles[2] = 0;
        //start motion thread
        motion_thread = new boost::thread(motion_thread_func, this);
    }

    steppermotor3::~steppermotor3(void)
    {
        thread_running = false;
        motion_thread->interrupt();
        stop();
        idle_cond.notify_all(); //its destructor will fail if threads are still waiting
        motion_thread->join();
        delete motion_thread;
        wait_till_ready();
        if(on)
        	modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CMD_1, 0);
    }

    bool steppermotor3::is_idle(void)
    {
        if(!on)
        	return false;

    	if(idle)
        {
        	boost::lock_guard<boost::mutex> lock(modbus_mutex);
        	return
				(modbus.read_u16(crd514_kd::slaves::MOTOR_1, crd514_kd::registers::STATUS_1) & crd514_kd::status1_bits::READY) &&
				(modbus.read_u16(crd514_kd::slaves::MOTOR_2, crd514_kd::registers::STATUS_1) & crd514_kd::status1_bits::READY) &&
				(modbus.read_u16(crd514_kd::slaves::MOTOR_3, crd514_kd::registers::STATUS_1) & crd514_kd::status1_bits::READY);
        }
        return false;
    }

    void steppermotor3::motion_thread_func(steppermotor3* owner)
    {
    	using namespace utils;

        try
        {
            while(owner->thread_running)
            {
                owner->queue_mutex.lock();
                
                if(!owner->motion_queue.empty())
                {
                    //get motion, convert and pop
                    motionf& mf = owner->motion_queue.front();
                    motioni mi;
                    motion_float_to_int(mi, mf);
                    owner->motion_queue.pop();
                    
                    owner->queue_mutex.unlock();
                    
                    if(owner->on)
                    {

						//write motion
						boost::lock_guard<boost::mutex> lock(owner->modbus_mutex);
						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_1, crd514_kd::registers::OP_SPEED, mi.speed[0], true);
						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_1, crd514_kd::registers::OP_POS, mi.angles[0], true);
						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_1, crd514_kd::registers::OP_ACC, mi.acceleration[0], true);
						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_1, crd514_kd::registers::OP_DEC, mi.deceleration[0], true);

						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_2, crd514_kd::registers::OP_SPEED, mi.speed[1], false);
						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_2, crd514_kd::registers::OP_POS, mi.angles[1], false);
						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_2, crd514_kd::registers::OP_ACC, mi.acceleration[1], true);
						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_2, crd514_kd::registers::OP_DEC, mi.deceleration[1], true);

						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_3, crd514_kd::registers::OP_SPEED, mi.speed[2], true);
						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_3, crd514_kd::registers::OP_POS, mi.angles[2], true);
						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_3, crd514_kd::registers::OP_ACC, mi.acceleration[2], true);
						owner->modbus.write_u32(crd514_kd::slaves::MOTOR_3, crd514_kd::registers::OP_DEC, mi.deceleration[2], true);

						//execute motion
						owner->wait_till_ready();

						owner->modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CMD_1, crd514_kd::cmd1_bits::EXCITEMENT_ON);
						owner->modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CMD_1, crd514_kd::cmd1_bits::EXCITEMENT_ON | crd514_kd::cmd1_bits::START);
						owner->modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CMD_1, crd514_kd::cmd1_bits::EXCITEMENT_ON);
                    }
                }
                else //empty
                {
                    owner->queue_mutex.unlock();
                    
                    //set idle bool
					owner->idle_mutex.lock();
					owner->idle = true;
					owner->idle_mutex.unlock();
					owner->idle_cond.notify_all();

                    //wait until not idle
                    boost::unique_lock<boost::mutex> lock(owner->idle_mutex);
                    while(owner->idle)
                    {
                    	owner->idle_cond.wait(lock);
                    }
                }
            }
        }
        catch(boost::thread_interrupted& ex)
        {
        }
        catch(modbus_exception& ex)
        {
            owner->exhandler(ex);
        }
    }
    

    void steppermotor3::wait_till_ready(void)
    {
    	static const crd514_kd::slaves::t slaves[] =
    		{ crd514_kd::slaves::MOTOR_1, crd514_kd::slaves::MOTOR_2, crd514_kd::slaves::MOTOR_3 };

        for(int i = 0; i < 3; i++)
        {
        	uint16_t status_1;
        	while(!((status_1 = modbus.read_u16(slaves[i], crd514_kd::registers::STATUS_1)) & crd514_kd::status1_bits::READY))
        	{
        		if((status_1 & crd514_kd::status1_bits::ALARM) ||
        		   (status_1 & crd514_kd::status1_bits::WARNING))
        		{
        			throw crd514_kd_exception(
        				slaves[i], status_1 & crd514_kd::status1_bits::WARNING,
        				status_1 & crd514_kd::status1_bits::ALARM);
        		}
        	}
        }
    }

    void steppermotor3::moveto(const motionf& mf, bool async)
    {
        if(!on)
        	return; //TODO fix! how will the user ever know his motion wasn't executed?

        if(mf.angles[0] <= min_angle || mf.angles[1] <= min_angle || mf.angles[2] <= min_angle ||
           mf.angles[0] >= max_angle || mf.angles[1] >= max_angle || mf.angles[2] >= max_angle)
        {
            throw std::out_of_range("one or more angles out of range");
        }

    	//push motion
        queue_mutex.lock();
        motion_queue.push(mf);
        queue_mutex.unlock();

        //unset idle bool
		idle_mutex.lock();
		idle = false;
		idle_mutex.unlock();
		idle_cond.notify_all();
        
        if(!async)
        {
            wait_for_idle();
        }
    }

    void steppermotor3::stop(void)
    {
        if(!on)
        	return;

    	boost::lock_guard<boost::mutex> queue_lock(queue_mutex);
        boost::lock_guard<boost::mutex> modbus_lock(modbus_mutex);
        
        try
        {
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CMD_1, crd514_kd::cmd1_bits::STOP);
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CMD_1, 0);
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CMD_1, crd514_kd::cmd1_bits::EXCITEMENT_ON);
        }
        catch(modbus_exception& ex)
        {
        	fprintf(stderr, "steppermotor3::stop failed:\nwhat(): %s\n", ex.what());
        }
     
        //empty queue (there doesn't seem to be a more elegant way)
        while(!motion_queue.empty())
        {
            motion_queue.pop();
        }
    }

    bool steppermotor3::wait_for_idle(long timeout)
    {
        if(!on)
        	return false;

    	if(idle)
    	{
    		return true;
    	}
    	else if(timeout > 0)
    	{
    		long timeout_end = utils::time_now() + timeout;
    		boost::unique_lock<boost::mutex> lock(idle_mutex);
    		while(!idle)
    		{
    			timeout = timeout_end - utils::time_now();
    			if(timeout > 0)
    			{
    				if(!idle_cond.timed_wait(lock, boost::posix_time::milliseconds(timeout)))
    				{	//timeout
    					return false;
    				}
    			}
    			else
    			{	//timeout
    				return false;
    			}
    		}

    		boost::lock_guard<boost::mutex> modbus_lock(modbus_mutex);
    		wait_till_ready();
    		return true;
    	}

    	//wait indefinitely
    	boost::unique_lock<boost::mutex> lock(idle_mutex);
		while(!idle)
		{
			idle_cond.wait(lock);
		}

		boost::lock_guard<boost::mutex> modbus_lock(modbus_mutex);
		wait_till_ready();
		return true;
    }

    void steppermotor3::motion_float_to_int(motioni& mi, const motionf& mf)
    {
        for(int i = 0; i < 3; i++)
        {
            mi.angles[i] = (uint32_t)((mf.angles[i] / crd514_kd::MOTOR_STEP_ANGLE));
            mi.speed[i] = (uint32_t)((mf.speed[i] / crd514_kd::MOTOR_STEP_ANGLE));
            mi.acceleration[i] = (uint32_t)((crd514_kd::MOTOR_STEP_ANGLE * 1000000000.0 / mf.acceleration[i]));
            mi.deceleration[i] = (uint32_t)((crd514_kd::MOTOR_STEP_ANGLE * 1000000000.0 / mf.deceleration[i]));
        }
    }

    void steppermotor3::power_off(void)
    {
        if(on){
            stop();
            boost::lock_guard<boost::mutex> lock(modbus_mutex);
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CMD_1, 0);
            on = false;
        }
    }

    void steppermotor3::moveto_within(const motionf & mf, double time, bool async)
    {
    	motionf newmf = mf;

    	newmf.speed[0] = fabs(current_angles[0] - mf.angles[0]) / time;
    	newmf.speed[1] = fabs(current_angles[1] - mf.angles[1]) / time;
    	newmf.speed[2] = fabs(current_angles[2] - mf.angles[2]) / time;

    	current_angles[0] = mf.angles[0];
    	current_angles[1] = mf.angles[1];
    	current_angles[2] = mf.angles[2];

    	moveto(newmf, async);
    }

    void steppermotor3::power_on(void)
    {
        if(!on){
            boost::lock_guard<boost::mutex> lock(modbus_mutex);
            //reset alarm
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::RESET_ALARM, 0);
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::RESET_ALARM, 1);
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::RESET_ALARM, 0);
            //set operating modes
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CMD_1, 0);
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::OP_POSMODE, 1);
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::OP_OPMODE, 0);
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::OP_SEQ_MODE + 0, 1);
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::OP_SEQ_MODE + 1, /*1*/
            0);
            //modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::OP_SEQ_MODE+2, 0); //loopback @ 2
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CMD_1, crd514_kd::cmd1_bits::EXCITEMENT_ON);
            //set motors limits
            modbus.write_u32(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CFG_POSLIMIT_POSITIVE, (uint32_t)((max_angle / crd514_kd::MOTOR_STEP_ANGLE)));
            modbus.write_u32(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CFG_POSLIMIT_NEGATIVE, (uint32_t)((min_angle / crd514_kd::MOTOR_STEP_ANGLE)));
            modbus.write_u32(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CFG_START_SPEED, 1);

            //clear counter
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CLEAR_COUNTER, 1);
            modbus.write_u16(crd514_kd::slaves::BROADCAST, crd514_kd::registers::CLEAR_COUNTER, 0);

            on = true;
        }
    }
}
