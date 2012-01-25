#pragma once

#include <ros/ros.h>
#include <deltarobotnode/motionSrv.h>
#include <datatypes/point3.hpp>
#include <iostream>

namespace cratedemo
{
class MotionWrapper
{
private:
	static const float OFFSET_X = 0;
	static const float OFFSET_Y = 0;
	static const float OFFSET_Z = 0;

	static const float FACTOR_X = 1;
	static const float FACTOR_Y = 1;
	static const float FACTOR_Z = 1;

public:
	deltarobotnode::motionSrv motions;

	void addMotion(const datatypes::point3f& p, double speed);
	bool callService(ros::ServiceClient& service);
	void print(std::ostream& os = std::cout);
	void addHack(datatypes::point3f& p);
};
}
