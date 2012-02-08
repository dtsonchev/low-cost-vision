#pragma once

#include <ros/ros.h>
#include <huniplacer/huniplacer.h>
#include <deltarobotnode/motionSrv.h>

#include "CrateModel.h"
#include "RangeModel.h"
#include "RobotModel.h"

using namespace huniplacer;

struct rosData {
	ros::NodeHandle* node;
	ros::ServiceClient moveToSrv;
	ros::Subscriber motionReceiver;
	ros::Subscriber crateReceiver;
	ros::Subscriber positionReceiver;
	ros::Subscriber topviewReceiver;
};

struct modelData{
	modelData():pivot(0,0,0){}
	inverse_kinematics_impl* ikmodel;
	imotor3* motor;
	effector_boundaries* eb;
	RobotModel* robot;
	RangeModel range;
	point3 pivot;
	float deltatime;
};

struct huniplacerData{
	rosData rosdata;
	modelData modeldata;
};

huniplacerData* getHuniplacerData();
