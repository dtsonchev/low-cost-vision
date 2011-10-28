#pragma once
#include <stdexcept>
#include <string>
#include <sstream>
#include <cerrno>

/**
 * @brief A throwable exception
 * @author Zep
 * @version 1.0
 * @date 10-2011
 */
class cameraException : public std::runtime_error
{
	public:
	/**
	* @brief function which is executed when ther is a exception
	* @param msg a string which contains the error
	*/
	cameraException(const std::string& msg):runtime_error(msg)
	{
	}
	/**
	* @brief distructor
	*/
	virtual ~cameraException(void) throw()
	{
	}
};

