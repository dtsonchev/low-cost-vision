/*#include "bounds_service_impl.h"
#include "utils.h"
#include <bitset>
#include <stack>

bool bounds_service_impl::is_valid(const point3& p)
{
	motionf mf;
	try
	{
		kinematics.point_to_motion(p, mf);
	}
	catch(inverse_kinematics_exception& ex)
	{
		return false;
	}

	for(int i = 0; i < 3; i++)
	{
		if(mf.angles[i] <= motors.get_min_angle() || mf.angles[i] >= motors.get_max_angle())
		{
			return false;
		}
	}

	return true;
}

bool bounds_service_impl::has_invalid_neighbours(const point3& p)
{
	for(int y = p.y-1; y <= p.y+1; y++)
	{
		for(int x = p.x-1; x <= p.x+1; x++)
		{
			if(x != p.x && y != p.y && !is_valid(point3(x, y, p.z)))
			{
				return true;
			}
		}
	}
	return false;
}

bounds_service_impl::bounds_service_impl(imotor3& motors, inverse_kinematics_model& kinematics) :
	motors(motors),
	kinematics(kinematics)
{}

bounds_service_impl::~bounds_service_impl(void)
{}

void bounds_service_impl::fill_response(bounds_srv::Request& req, bounds_srv::Response& res)
{
	typedef std::pair<int, int> coord;
	const int width = measures::MAX_X - measures::MIN_X;
	const int height = measures::MAX_Y - measures::MIN_Y;

	//std::bitset<width * height> set;
	std::vector<bool> set(width * height, false);
	std::stack<coord> cstack;

	if(is_valid(point3(0, 0, req.z)))
	{
		for(int x = 1; x < measures::MAX_X; x++)
		{
			if(!is_valid(point3(x, 0, req.z)))
			{
				cstack.push(coord(x-1, 0));
				set[x-1] = true;
				res.x.push_back(x-1);
				res.y.push_back(0);
				break;
			}
		}

		while(!cstack.empty())
		{
			coord c = cstack.top();
			cstack.pop();

			for(int y = c.second-1; y <= c.second+1; y++)
			{
				for(int x = c.first-1; x <= c.first+1; x++)
				{
					int set_n = (y+measures::MAX_Y) * width + (x+measures::MAX_X);
					if(is_valid(point3(x, y, req.z))
					&& !((x < measures::MAX_X && x >= measures::MIN_X && y < measures::MAX_Y && y >= measures::MIN_Y)
					&& set[set_n])
					&& has_invalid_neighbours(point3(x, y, req.z)))
					{
						set[set_n] = true;
						cstack.push(coord(x, y));
						res.x.push_back(x);
						res.y.push_back(y);
					}
				}
			}
		}
	}
}
*/
