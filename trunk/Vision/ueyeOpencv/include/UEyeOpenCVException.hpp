#pragma once
#include <sstream>
#include <exception>
#include <iostream>
class UeyeOpenCVException:  public std::exception {
public:
	UeyeOpenCVException(unsigned int cam, int err) {
		exceptionId = err;
	}
	const char * what() const throw () {
		std::stringstream ss;
		ss << "UeyeOpenCVException on camera " << cam <<", with exit code:\t" << exceptionId;

		return ss.str().c_str();
	}
private:
	unsigned int cam;
	int exceptionId;
};
