#pragma once
#include <ueye.h>
#include <sstream>
#include <exception>
#include <iostream>
//TODO zorg dat what() de bijbehorende string van de errorcode print
class UeyeOpenCVException : public std::exception {
private:
	HIDS cam;
	int exceptionId;
public:
	UeyeOpenCVException(HIDS cam, int err) {
		exceptionId = err;
	}
	const char * what() const throw () {
		std::stringstream ss;
		ss << "UeyeOpenCVException on camera " << cam <<", with exit code:\t" << exceptionId;

		return ss.str().c_str();
	}
	HIDS getCam()
	{
		return cam;
	}
	int getExceptionId()
	{
		return exceptionId;
	}
};
