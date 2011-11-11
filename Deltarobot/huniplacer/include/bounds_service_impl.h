/*#pragma once

#include "huniplacer.h"
#include <huniplacer/bounds_srv.h>

using namespace huniplacer;

class bounds_service_impl
{
	private:
		imotor3& motors;
		inverse_kinematics_model& kinematics;

		bool is_valid(const point3& p);
		bool has_invalid_neighbours(const point3& p);

	public:
		bounds_service_impl(imotor3& motors, inverse_kinematics_model& kinematics);
		~bounds_service_impl(void);

		void fill_response(bounds_srv::Request& req, bounds_srv::Response& res);
};
*/
