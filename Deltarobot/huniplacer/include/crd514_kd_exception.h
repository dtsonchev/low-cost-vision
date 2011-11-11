#pragma once

#include "CRD514_KD.h"

#include <stdexcept>
#include <string>
#include <sstream>

namespace huniplacer
{
	/**
	 * @brief raised if the alarm flag
	 **/
	class crd514_kd_exception : public std::runtime_error
	{
		private:
			const crd514_kd::slaves::t slave;
			const bool warning, alarm;
			std::string message;

		public:
			crd514_kd_exception(const crd514_kd::slaves::t slave, const bool warning, const bool alarm) :
				std::runtime_error(""),
				slave(slave),
				warning(warning),
				alarm(alarm)
			{
				std::stringstream ss;
				ss << "controller " << (int)slave << ": warning=" << (int)warning << " alarm=" << (int)alarm;
				message = ss.str();
			}

			~crd514_kd_exception() throw()
			{ }

			const char* what()
			{
				return message.c_str();
			}

			crd514_kd::slaves::t get_slave(void)
			{
				return slave;
			}

			bool get_warning(void)
			{
				return warning;
			}

			bool get_alarm(void)
			{
				return alarm;
			}
	};
}
