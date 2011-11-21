#pragma once

#include <queue>
#include <boost/thread.hpp>

#include <huniplacer/motion.h>
#include <huniplacer/modbus_exception.h>
#include <huniplacer/modbus_ctrl.h>
#include <huniplacer/imotor3.h>

namespace huniplacer
{
	/// @brief exception handler to which modbus_exception's will be passed that occur in motion_thread
    typedef void (*motion_thread_exception_handler)(std::runtime_error& ex);

    /// @brief implementation of imotor3 for steppermotors
    class steppermotor3 : public imotor3
    {
        private:
            std::queue<motionf> motion_queue;
            bool thread_running;
            double current_angles[3];
            
            bool idle;
            boost::mutex idle_mutex;
            boost::condition_variable idle_cond;
            
            boost::mutex queue_mutex;
            boost::mutex modbus_mutex;
            
            const double min_angle;
            const double max_angle;
            
            modbus_ctrl modbus;
            
            motion_thread_exception_handler exhandler;
            boost::thread* motion_thread;
            
            volatile bool powered_on;

            /**
             * @brief function passed to motion_thread
             * @param owner pointer to object that start the thread
             * @note (un)locks queue_mutex
             * @note waits for queue_newitem_flag
             * @note signals queue_empty_flag
             **/
            static void motion_thread_func(steppermotor3* owner);
        
            void wait_till_ready(void);
            
            /**
             * @brief converts a motion in floating point notation to values for the motor controllers
             * @param mi angles(0-360), speed(?-?), acceleration(?-?), deceleration(?-?)
             * @param mf angles(0-5000), speed(?-?), acceleration(?-?), deceleration(?-?)
             **/
            static void motion_float_to_int(motioni& mi, const motionf& mf);
        
        public:
            steppermotor3(modbus_t* context, double min_angle, double max_angle, motion_thread_exception_handler exhandler);
            virtual ~steppermotor3(void);
            
            /**
             * @brief pushes a motion into the motion queue
             * @param mf a motion
             * @param async if false: the calling thread will wait until the motion queue is empty
             * @note signals queue_newitem_flag
             * @note may wait for queue_empty_flag
             **/
            void moveto(const motionf& mf, bool async = true);

            /**
             * @brief same as moveto, but rotates to an angle within a certain time.
             * @param time time in seconds that the motors will take to rotate to the given angle
             * speed members of given motion is ignored
             */
            void moveto_within(const motionf& mf, double time, bool async);

            /**
             * @brief stops the motors & clears the motion queue
             * @note (un)locks queue_mutex
             **/
            void stop(void);
            bool wait_for_idle(long timeout = 0);
            bool is_idle(void);
            
            void power_off(void);

            void power_on(void);

            bool is_powerd_on(void);

            inline double get_min_angle(void) { return min_angle; }
            inline double get_max_angle(void) { return max_angle; }
    };
}
