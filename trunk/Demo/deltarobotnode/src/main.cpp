#include <iostream>
#include <stdexcept>
#include <huniplacer/huniplacer.h>
#include <gripper/gripper.h>
#include "ros/ros.h"
#include "deltarobotnode/motion.h"
#include "deltarobotnode/stop.h"
#include "deltarobotnode/gripper.h"
#include "deltarobotnode/error.h"
#include "deltarobotnode/error.h"

using namespace huniplacer;
//using namespace deltarobot;

static huniplacer::deltarobot * robot;
static gripper * grip;
static ros::Publisher * pub;

static void modbus_exhandler(std::exception& ex)
{
	deltarobotnode::error msg;
	std::stringstream ss;
	ss << "runtime error of type "<< typeid(ex).name()<<" in delta robot" << std::endl;
	ss <<"what(): " << ex.what()<<std::endl;
	msg.errorMsg = ss.str();
	msg.errorType = 1;
	pub->publish(msg);
}

bool moveTo(deltarobotnode::motion::Request &req,
		deltarobotnode::motion::Response &res)
{
	ROS_INFO("-- moveTo");
	res.succeeded = true;
	try
	{
		unsigned int n;
		for(n = 0; n < req.x.size() - 1; n++)
		{
			if(!robot->check_path(point3(req.x[n],req.y[n],req.z[n]),point3(req.x[n+1],req.y[n+1],req.z[n+1])))
			{
				res.succeeded = false;
				return true;
			}
		}
		for(n = 0; n < req.x.size(); n++)
		{	
			ROS_INFO("moveTo: (%f, %f, %f) speed=%f", req.x[n], req.y[n], req.z[n], req.speed[n]);
			robot->moveto(point3(req.x[n],req.y[n],req.z[n]),req.speed[n]);
		}
		robot->wait_for_idle();
	}
	catch(std::runtime_error& ex)
	{
		deltarobotnode::error msg;
		std::stringstream ss;
		ss << "runtime error of type "<< typeid(ex).name()<<" in delta robot" << std::endl;
		ss <<"what(): " << ex.what()<<std::endl;
		msg.errorMsg = ss.str();
		msg.errorType = 2;
		pub->publish(msg);
		res.succeeded = false;
		ROS_ERROR("moveTo: %s", ss.str().c_str());
	}
	return true;
}

bool enableGripper(deltarobotnode::gripper::Request &req,
		deltarobotnode::gripper::Response &res)
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
		deltarobotnode::error msg;
		std::stringstream ss;
		ss << "runtime error of type "<< typeid(ex).name()<<" in delta robot" << std::endl;
		ss <<"what(): " << ex.what()<<std::endl;
		msg.errorMsg = ss.str();
		msg.errorType = 3;
		pub->publish(msg);
	}
	return true;
}

bool stop(deltarobotnode::stop::Request &req,
		deltarobotnode::stop::Response &res)
{
	robot->stop();
	return true;
}

void calibrateMotor(steppermotor3& motors, int motorIndex){
	std::cout << "Calibrating motor number " << motorIndex << std::endl;
	std::cout << "Enter motor rotation (radians) or 0 to finish." << std::endl;

	double angle = utils::rad(45);
	double deltaAngle = 0.0;
	do {
		std::cout << "deltaAngle: ";
		std::cin >> deltaAngle;
		angle += deltaAngle;
		if(motorIndex == 0){
			motors.moveto(motionf(angle, -measures::MOTOR2_DEVIATION, -measures::MOTOR3_DEVIATION, 10, 10, 10, 360, 360, 360, 360, 360, 360), false);
		} else if(motorIndex == 1){
			motors.moveto(motionf(-measures::MOTOR1_DEVIATION,angle,-measures::MOTOR3_DEVIATION, 10, 10, 10, 360, 360, 360, 360, 360, 360), false);
		} else {
			motors.moveto(motionf(-measures::MOTOR1_DEVIATION, -measures::MOTOR2_DEVIATION, angle, 10, 10, 10, 360, 360, 360, 360, 360, 360), false);
		}
	} while(deltaAngle != 0.0);

	double curAngle;
	std::cout << "Real motor angle (degrees): ";
	std::cin >> curAngle;

	double angles[] = {-measures::MOTOR1_DEVIATION, -measures::MOTOR2_DEVIATION, -measures::MOTOR3_DEVIATION};
	angles[motorIndex] = utils::rad(curAngle);
	motors.override_current_angles(angles);

	motors.moveto(motionf(-measures::MOTOR1_DEVIATION, -measures::MOTOR2_DEVIATION, -measures::MOTOR3_DEVIATION, 10, 10, 10, 360, 360, 360, 360, 360, 360), false);
}

void calibrateAllMotors(steppermotor3& motors){
	motors.set_max_angle(utils::rad(120));

	for(int i = 0; i < 3; i++){
		calibrateMotor(motors, i);
	}

	motors.set_max_angle(measures::MOTOR_ROT_MAX);
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

	double deviation[3] = {measures::MOTOR1_DEVIATION, measures::MOTOR2_DEVIATION, measures::MOTOR3_DEVIATION};
	steppermotor3 motors(modbus_rtu, measures::MOTOR_ROT_MIN, measures::MOTOR_ROT_MAX, modbus_exhandler, deviation);
	motors.power_on();

	calibrateAllMotors(motors);

	robot = new huniplacer::deltarobot(kinematics, motors);
	robot->generate_boundaries(2);
	robot->power_on();

	ros::init(argc, argv, "Deltarobot");
	ros::NodeHandle n;
	ros::ServiceServer service1 = n.advertiseService("moveTo", moveTo);
	ros::ServiceServer service2 = n.advertiseService("enableGripper", enableGripper);
	ros::ServiceServer service3 = n.advertiseService("stop", stop);

	ros::Publisher pubTemp = n.advertise<deltarobotnode::error>("deltaError", 100);
	pub = &pubTemp;

	ros::spin();

    robot->wait_for_idle();
	grip->disconnect();

	return 0;
}

