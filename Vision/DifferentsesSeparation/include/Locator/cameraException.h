//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        DifferentsesSeparation
// File:           cameraException.h
// Description:    A header file for camera exceptions
// Author:         Glenn Meerstra & Zep Mouris
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of DifferentsesSeparation.
//
// DifferentsesSeparation is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// DifferentsesSeparation is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with DifferentsesSeparation.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************
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

