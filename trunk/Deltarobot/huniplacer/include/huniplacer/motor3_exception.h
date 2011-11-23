#pragma once

#include <stdexcept>
#include <string>

namespace huniplacer
{
	class motor3_exception : public std::runtime_error
	{
		public:
			motor3_exception(const std::string& message = std::string()) :
				std::runtime_error(message)
			{
			}

			virtual ~motor3_exception(void) throw()
			{
			}
	};
}
