//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        DeltaRobotNode
// File:           main.cpp
// Description:    Contains the whole delta robot ROSnode
// Author:         Kasper van Nieuwland en Zep Mouris
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of DeltaRobotNode.
//
// DeltaRobotNode is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// DeltaRobotNode is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with DeltaRobotNode.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************
#include <iostream>
#include <stdexcept>
#include <huniplacer/huniplacer.h>
#include <gripper/gripper.h>
#include "ros/ros.h"
#include "deltarobotnode/motions.h"
#include "deltarobotnode/stop.h"
#include "deltarobotnode/gripper.h"
#include "deltarobotnode/error.h"
#include "deltarobotnode/motionSrv.h"

using namespace huniplacer;

static huniplacer::deltarobot * robot;
static gripper * grip;
static ros::Publisher * pub;
static ros::Publisher * pubDeltaPos;

//callback function that gets called by the deltarobot thread when an exception occured in it
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

bool moveTo(deltarobotnode::motionSrv::Request &req,
		deltarobotnode::motionSrv::Response &res)
{
	ROS_INFO("-- moveTo");
	res.succeeded = true;
	try
	{
		unsigned int n;
		for(n = 0; n < req.motions.x.size() - 1; n++)
		{
			if(!robot->check_path(point3(req.motions.x[n],req.motions.y[n],req.motions.z[n]),point3(req.motions.x[n+1],req.motions.y[n+1],req.motions.z[n+1])))
			{
				res.succeeded = false;
				return true;
			}
		}
		for(n = 0; n < req.motions.x.size(); n++)
		{	
			ROS_INFO("moveTo: (%f, %f, %f) speed=%f", req.motions.x[n], req.motions.y[n], req.motions.z[n], req.motions.speed[n]);
			robot->moveto(point3(req.motions.x[n],req.motions.y[n],req.motions.z[n]),req.motions.speed[n]);
		}
		deltarobotnode::motions msg;
		msg = req.motions; 		
		pubDeltaPos->publish(msg);
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

bool checkTo(deltarobotnode::motionSrv::Request &req,
		deltarobotnode::motionSrv::Response &res)
{
	res.succeeded = true;
	try
	{
		unsigned int n;
		for(n = 0; n < req.motions.x.size() - 1; n++)
		{
			if(!robot->check_path(point3(req.motions.x[n],req.motions.y[n],req.motions.z[n]),point3(req.motions.x[n+1],req.motions.y[n+1],req.motions.z[n+1])))
			{
				res.succeeded = false;
				return true;
			}
		}
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
		ROS_ERROR("checkTo: %s", ss.str().c_str());
	}
	return true;
}

static bool gripper_status = false;
static bool overheated = false;
bool enableGripper(deltarobotnode::gripper::Request &req,
		deltarobotnode::gripper::Response &res)
{
	try
	{
		res.succeeded = true;
		if(req.enabled)
		{
			if(!overheated)
			{
				gripper_status = true;
				//grip->grab();
			}
			else
			{
				ROS_WARN("Tried to turn on gripper, but it's valve is currently overheated. Ignoring request");
				res.succeeded = false;
			}
		} else {
			gripper_status = false;
			//grip->release();
		}
		
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
	std::cout << "Enter motor rotation (degrees) or 0 to finish." << std::endl;

	double angle = utils::rad(45);
	double deltaAngle = 0.0;
	do {
		std::cout << "deltaAngle: ";
		std::cin >> deltaAngle;
		deltaAngle = utils::rad(deltaAngle);
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
	ros::ServiceServer service4 = n.advertiseService("checkTo", checkTo);

	ros::Publisher pubTemp = n.advertise<deltarobotnode::error>("deltaError", 100);
	ros::Publisher pubDeltaPosTemp= n.advertise<deltarobotnode::motions>("pubDeltaPos", 100);
	pub = &pubTemp;
	pubDeltaPos = &pubDeltaPosTemp;

	ros::Time gripper_went_on;
	ros::Time got_overheated;
	bool previous_gripper_status = gripper_status;
	static const int MAX_GRIPPER_ON = 60; //sec
	static const int COOLDOWN_DURATION = 3*60; //sec
	
	while(ros::ok())
	{
		
		//prevent the watchdog to trigger by sending the same command again
		if(gripper_status)
			grip->grab();
		else
			grip->release();

		if(!previous_gripper_status && gripper_status)
		{
			gripper_went_on = ros::Time::now();
		}
		else if(previous_gripper_status && gripper_status && (ros::Time::now() - gripper_went_on).toSec() > MAX_GRIPPER_ON)
		{			
			ROS_WARN("Gripper valve was turned on for longer than %d seconds. Gripper will be forced to turn off now to prevent overheating", MAX_GRIPPER_ON);
			overheated = true;
			got_overheated = ros::Time::now();
			grip->release();
			previous_gripper_status = gripper_status = false;
		}
		else if(overheated && (ros::Time::now() - got_overheated).toSec() > COOLDOWN_DURATION)
		{
			ROS_WARN("Gripper valve cooled down");
			overheated = false;
		}

		previous_gripper_status = gripper_status;
		ros::spinOnce();
	}

	grip->release();

    robot->wait_for_idle();
	grip->disconnect();

	return 0;
}

