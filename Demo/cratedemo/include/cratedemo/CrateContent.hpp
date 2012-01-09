//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        cratedemo
// File:           CrateContent.hpp
// Description:    Representation of an object which can be placed in a crate.
// Author:         Wouter Langerak
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
#include <datatypes/point3.hpp>
#include <datatypes/size3.hpp>
/*
gripPoint is the offset of the most left point of the crateContent.
For example in the picture(of an object seen from above) the gripPoint(*) has an offset of 7,0,-4.
0,0,0 ---------------X
|██████|██████
| █████|█████
|  ████|████
|------*████
|  █████████
|    █████
|     ███
|      █
Z
 */
/**
 * Representation of an object which can be placed in a crate.
 */

namespace cratedemo {
class CrateContent {

public:
	/**
	 * Returns the gripPoint
	 * @return
	 */
	datatypes::point3f getGripPoint() const { return gripPoint; }
	/**
	 * Returns the size
	 * @return
	 */
	datatypes::size3f getSize() const { return size; }
	/**
	 * Returns the size.
	 * @return
	 */
	double getWeight() const {return weight; }

protected:
	/**
	 *
	 * @param gripPoint gripPoint is the offset of the most left point of the crateContent.
	 * @param size size of the object
	 * @param weight weight of the object
	 */
	CrateContent(datatypes::point3f gripPoint,datatypes::size3f size, double weight): gripPoint(gripPoint), size(size), weight(weight) {}

private:
	datatypes::point3f gripPoint; //in mm
	datatypes::size3f size; //in mm
	double weight; //in gram
};
}
