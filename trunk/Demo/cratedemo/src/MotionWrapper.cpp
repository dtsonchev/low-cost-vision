#include <cratedemo/MotionWrapper.hpp>

namespace cratedemo
{
	void MotionWrapper::addMotion(const datatypes::point3f& p, double speed)
	{

		datatypes::point3f point = p;
		//addHack(point);

		motions.request.x.push_back(point.x - 0.475);// * FACTOR_X + OFFSET_X);
		motions.request.y.push_back(point.y - 0.911);// * FACTOR_Y + OFFSET_Y);
		motions.request.z.push_back(point.z);// * FACTOR_Z + OFFSET_Z);
		motions.request.speed.push_back(speed);
	}

	void MotionWrapper::addHack(datatypes::point3f& p){
		//p.x -= 0.475;
		//p.y -= 0.911;

		if(p.x > 0 && p.y > 0){
			p.x += p.x * ((20 - (20.346-0.475))/20.0);
			p.y += p.y * ((20 - (21.961-0.911))/20.0);
		}else if(p.x > 0 && p.y < 0){
			p.x += p.x * ((20 - (22.119-0.475))/20.0);
			p.y += p.y * ((-20 - (-19.699-0.911))/-20.0);
		}else if(p.x < 0 && p.y < 0){
			p.x += p.x * ((-20 - (-19.731-0.475))/-20.0);
			//p.y += p.y * ((-20 - (-20.51-0.911))/-20.0);
		}else if(p.x < 0 && p.y > 0){
			p.x += p.x * ((-20 - (-20.596-0.475))/-20.0);
			p.y += p.y * ((20 - (20.911-0.911))/20.0);
		}
	}

	bool MotionWrapper::callService(ros::ServiceClient& service)
	{
		service.call(motions);
		return motions.response.succeeded;
	}

	void MotionWrapper::print(std::ostream& os)
	{
		os << "motions:" << std::endl;
		for(size_t i = 0; i < motions.request.x.size(); i++)
		{
			#define P motions.request
			os << "  i=" << i << " pos=(" << P.x.at(i) << ',' << P.y.at(i) << ',' << P.z.at(i) << ") speed=" << P.speed.at(i) << std::endl;
			#undef P
		}
		os << std::endl;
	}
}
