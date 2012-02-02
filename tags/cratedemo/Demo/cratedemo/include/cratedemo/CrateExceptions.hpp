//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        cratedemo
// File:           CrateExceptions.hpp
// Description:    Exceptions
// Author:         Wouter Langerak & Lukas Vermond
// Notes:
//
// License:        GNU GPL v3
//
// This file is part of cratedemo.
//
// cratedemo is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// cratedemo is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with cratedemo.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#pragma once
#include <exception>
namespace cratedemo {

/**
 * Exception which can be thrown when a location in a crate is (unexpectedly) empty.
 */
 class LocationIsEmptyException : public std::exception {
 public:
	 LocationIsEmptyException() throw() {}
	 virtual ~LocationIsEmptyException() throw() {}
	 virtual const char* what() const throw() {
		 return "Location Is Empty";
	 }
 };
 /**
  * Exception which can be thrown when a location in a crate is (unexpectedly) full.
  */
 class LocationIsFullException : public std::exception {
 public:
	 LocationIsFullException() throw() {}
	 virtual ~LocationIsFullException() throw() {}
	 virtual const char* what() const throw() {
		 return "Location Is Full";
	 }
 };
}
