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
class variableException: public std::runtime_error {
public:
	/**
	 * @brief function which is executed when ther is a exception
	 * @param msg a string which contains the error
	 */
	variableException(const std::string& msg) :
			runtime_error(msg) {
	}
	/**
	 * @brief destructor
	 */
	virtual ~variableException(void) throw () {
	}
};

