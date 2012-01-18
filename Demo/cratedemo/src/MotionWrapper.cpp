#pragma once

#include <cratedemo/MotionWrapper.hpp>

namespace cratedemo
{
	void MotionWrapper::addMotion(const datatypes::point3lf& p, double speed)
	{
		motions.request.x.push_back(p.x * FACTOR_X + OFFSET_X);
		motions.request.y.push_back(p.y * FACTOR_Y + OFFSET_Y);
		motions.request.z.push_back(p.z * FACTOR_Z + OFFSET_Z);
		motions.request.speed.push_back(speed);
	}

	bool MotionWrapper::callService(ServiceClient& service)
	{
		service.call(motions);
		return motions.response.succeeded;
	}

	void MotionWrapper::print(std::ostream& os)
	{
		os << "motions:" << std::endl;
		for(size_t i = 0; i < motions.size(); i++)
		{
			#define P motions.request
			os << "  i=" << i << " pos=(" << P.x.at(i) << ',' << P.y.at(i) << ',' << P.z.at(i) << ") speed=" << P.speed.at(i) << std::endl;
			#undef P
		}
		os << std::endl;
	}
}
