#pragma once
#include <exception>
namespace cratedemo {
 class LocationIsEmptyException : public std::exception {
 public:
	 LocationIsEmptyException() {}
	 virtual ~LocationIsEmptyException() throw() {}
	 virtual const char* what() const throw() {
		 return "Location Is Empty";
	 }
 };
 class LocationIsFullException : public std::exception {
 public:
	 LocationIsFullException() {}
	 virtual ~LocationIsFullException() throw() {}
	 virtual const char* what() const throw() {
		 return "Location Is Full";
	 }
 };
}
