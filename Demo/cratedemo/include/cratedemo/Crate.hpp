//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        cratedemo
// File:           Crate.hpp
// Description:    Base class for crates.
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
#include <string.h>
#include <string>
#include <cratedemo/CrateContent.hpp>
#include <datatypes/point2.hpp>
#include <datatypes/point3.hpp>
#include <datatypes/size3.hpp>

namespace cratedemo {
/**
 * Base class for crates.
 */
class Crate {

public:
	virtual ~Crate();
	/**
	 * Puts a CrateContent* in the list of containing items on location index.
	 * @param index Location of the to be inputed CrateContent*.
	 * @param crateContent Pointer of the to be inputed CrateContent.
	 */
	void put(size_t index, CrateContent* crateContent);
	/**
	 *
	 */
	CrateContent* get(size_t index) const;
	/**
	 *
	 */
	virtual datatypes::point3f getContainerLocation(size_t index) const = 0;
	/**
	 * Returns the location of the object on index.
	 * @param index
	 * @return the location of the object on index.
	 */
	virtual datatypes::point3f getCrateContentGripLocation(size_t index) const = 0;
	/**
	 * Remove the content on location index.
	 * @param index
	 */
	void remove(size_t index);
	/**
	 * Returns if the Crate is empty.
	 * @return
	 */
	bool isEmpty() const;

	const std::string& getName(void) const;
protected:
	Crate(std::string name, datatypes::point2f position, float angle, datatypes::size3f size, std::vector<CrateContent*>& crateContent);

	std::string name;
public:
	datatypes::point2f position;
	float angle;
protected:
	datatypes::size3f size;
	std::vector<CrateContent*>& data;
	static const float TABLE_HEIGHT = -200;

};
}
