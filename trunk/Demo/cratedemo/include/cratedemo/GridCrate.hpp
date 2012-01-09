//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        cratedemo
// File:           GridCrate.hpp
// Description:    Symmetric crate containing balls.
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
#include <cratedemo/Crate.hpp>
#include <string>
namespace cratedemo {
/**
 * Symmetric crate containing balls
 */
class GridCrate : public Crate {
public:
	/**
	 * Crate with on type of content arranged in a grid
	 * @param name name of the crate
	 * @param position position of the crate
	 * @param angle rotation in the middle of the crate
	 * @param size of the crate
	 * @param maxNumberOfObjects Maximal number of objects that the crate can hold.
	 * @param gridWidth Number of objects in x-direction in the crate
	 * @param gridHeight Number of objects in y-direction in the crate
	 * @param distanceToSide distance (in mm) from the side of the crate to the border of the first element.
	 * @param distanceToNext distance (in mm) between two object containers
	 * @param radiusOfBallContainer Radius of the ball Container
	 * @param bottomThickness
	 */
	GridCrate(
		std::string name,
		datatypes::point2f position,
		float angle,
		datatypes::size3f size,
		size_t maxNumberOfObjects,
		size_t gridWidth,
		size_t gridHeight,
		double distanceToSide,
		double distanceToNext,
		double radiusOfBallContainer,
		double bottomThickness);
	datatypes::point3f getCrateContentGripLocation(size_t index);

private:
	size_t gridWidth;
	size_t gridHeight;
	double distanceToSide;
	double distanceToNext;
	double bottomThickness;
	double radiusOfBallContainer;
};
}
