//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        FGBGSeparation
// File:           variableException.h
// Description:    library can be trained with images. with normal images and FGBG images where white means forground and black means background after training the library can generate the FGBG images self. this is a exception means there is one variable out of range
// Author:         Glenn Meerstra & Zep Mouris
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of FGBGSeparation.
//
// FGBGSeparation is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// FGBGSeparation is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with FGBGSeparation.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************
#pragma once

#include <stdexcept>
#include <string>
#include <sstream>
#include <cerrno>

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

