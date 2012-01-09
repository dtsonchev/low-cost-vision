//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        cratedemo
// File:           CrateDemo.hpp
// Description:    Framework for a demo with crates.
// Author:         Lukas Vermond
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

#include <string>
#include <cratedemo/Crate.hpp>

namespace cratedemo
{
/**
 * Framework for a demo with crates.
 */
class CrateDemo
{
protected:
	CrateDemo(
		const std::string& deltaGrip, const std::string& deltaStop, const std::string& deltaMove, const std::string& deltaError,
		const std::string& visionGetCrate, const std::string& visionGetAllCrates, const std::string& visionError);

public:
	/**
	 *Deconstructor
	 */
	virtual ~CrateDemo();
/**
 * Method which is called after the (vision) system detects a new crate.
 * @param crate
 */
	virtual void onNewCrate(Crate& crate) = 0;

	/**
	 * Method which is called after the (vision) system detects that a crate is moved.
	 * @param crate
	 */
	virtual void onCrateMove(Crate& crate) = 0;
/**
 * Updates
 */
	void update();
};
}
