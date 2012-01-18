#pragma once

#include <ros.h>
#include <deltarobotnode/motion.h>
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
	deltarobotnode::motion motions;

	void addMotion(const datatypes::point3lf& p, double speed);
	bool callService(ServiceClient& service);
	void print(std::ostream& os = std::cout);
};
}
