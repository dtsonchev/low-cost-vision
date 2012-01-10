#include <iostream>
#include <stdexcept>
#include <huniplacer/huniplacer.h>
#include <gripper/gripper.h>
#include "ros/ros.h"
#include "deltarobot/motion.h"
#include "deltarobot/stop.h"
#include "deltarobot/gripper.h"
#include "deltarobot/error.h"

using namespace huniplacer;
//using namespace deltarobot;

static huniplacer::deltarobot * robot;
static gripper * grip;
static ros::Publisher * pub;

static void modbus_exhandler(std::runtime_error& ex)
{
	deltarobot::error msg;
	std::stringstream ss;
	ss << "runtime error of type "<< typeid(ex).name()<<" in delta robot" << std::endl;
	ss <<"what(): " << ex.what()<<std::endl;
	msg.errorMsg = ss.str();
	msg.errorType = 1;
	pub->publish(msg);
}

bool moveTo(deltarobot::motion::Request &req,
		deltarobot::motion::Response &res)
{
	res.succeeded = true;
	try
	{
		int n;
		for(int n = 0; n < req.x.size() - 1; n++)
		{
			if(!robot->check_path(point3(req.x[n],req.y[n],req.z[n]),point3(req.x[n+1],req.y[n+1],req.z[n+1])))
			{
				res.succeeded = false;
				return true;
			}
		}
		for(n = 0; n < req.x.size() - 1; n++)
		{
			robot->moveto(point3(req.x[n],req.y[n],req.z[n]),req.speed[n], true);
		}
		robot->moveto(point3(req.x[n],req.y[n],req.z[n]),req.speed[n], false);
	}
	catch(std::runtime_error& ex)
	{
		deltarobot::error msg;
		std::stringstream ss;
		ss << "runtime error of type "<< typeid(ex).name()<<" in delta robot" << std::endl;
		ss <<"what(): " << ex.what()<<std::endl;
		msg.errorMsg = ss.str();
		msg.errorType = 2;
		pub->publish(msg);
		res.succeeded = false;
	}
	return true;
}

bool enableGripper(deltarobot::gripper::Request &req,
		deltarobot::gripper::Response &res)
{
	try
	{
		if(req.enabled){
			grip->grab();
		} else {
			grip->release();
		}
		res.succeeded = true;
	}
	catch(std::runtime_error& ex)
	{
		res.succeeded = false;
		deltarobot::error msg;
		std::stringstream ss;
		ss << "runtime error of type "<< typeid(ex).name()<<" in delta robot" << std::endl;
		ss <<"what(): " << ex.what()<<std::endl;
		msg.errorMsg = ss.str();
		msg.errorType = 3;
		pub->publish(msg);
	}
	return true;
}

bool stop(deltarobot::stop::Request &req,
		deltarobot::stop::Response &res)
{
	robot->stop();
	return true;
}

int main(int argc, char** argv)
{
	grip = new gripper("192.168.0.2", 502);
	grip->connect();

	inverse_kinematics_impl kinematics(
		measures::BASE,
		measures::HIP,
		measures::EFFECTOR,
		measures::ANKLE,
		measures::HIP_ANKLE_ANGLE_MAX);

	modbus_t* modbus_rtu = modbus_new_rtu(
		"/dev/ttyS0",
		crd514_kd::rtu_config::BAUDRATE,
		crd514_kd::rtu_config::PARITY,
		crd514_kd::rtu_config::DATA_BITS,
		crd514_kd::rtu_config::STOP_BITS);

	double deviation[3] = {0,0,0};
	steppermotor3 motors(modbus_rtu, utils::rad(-45), utils::rad(75), modbus_exhandler, deviation);
	robot = new huniplacer::deltarobot(kinematics, motors);
	robot->generate_boundaries(2);
	robot->power_on();

	ros::init(argc, argv, "Deltarobot");
	ros::NodeHandle n;
	ros::ServiceServer service1 = n.advertiseService("moveTo", moveTo);
	ros::ServiceServer service2 = n.advertiseService("enableGripper", enableGripper);
	ros::ServiceServer service3 = n.advertiseService("stop", stop);

	ros::Publisher pubTemp = n.advertise<deltarobot::error>("deltaError", 100);
	pub = &pubTemp;

	ros::spin();

    robot->wait_for_idle();
	grip->disconnect();

	return 0;
}

