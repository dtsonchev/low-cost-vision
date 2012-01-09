//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        cratedemo
// File:           GridCrate.cpp
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

#include <cratedemo/GridCrate.hpp>
#include <cratedemo/Crate.hpp>
#include <cratedemo/CrateExceptions.hpp>
#include <datatypes/point2.hpp>
#include <datatypes/point3.hpp>
#include <datatypes/size3.hpp>
#include <string>

namespace cratedemo {

GridCrate::GridCrate(std::string name, datatypes::point2f position, float angle, datatypes::size3f size, size_t maxNumberOfObjects, size_t gridWidth, size_t gridHeight, double distanceToSide, double distanceToNext, double radiusOfBallContainer, double bottomThickness) :
		Crate(name, position, angle, size, maxNumberOfObjects) {

	this->gridWidth = gridWidth;
	this->gridHeight = gridHeight;
	this->distanceToSide = distanceToSide;
	this->distanceToNext = distanceToNext;
	this->bottomThickness = bottomThickness;
	this->radiusOfBallContainer = radiusOfBallContainer;
}

datatypes::point3f GridCrate::getCrateContentGripLocation(size_t index) {
	using datatypes::point2f;
	using datatypes::point3f;

	if (data[index] == NULL) {
		throw cratedemo::LocationIsEmptyException();
	}

	point2f location2D;
	location2D.x = -(size.width / 2) + (index % gridWidth) * (2 * radiusOfBallContainer + distanceToNext) + distanceToSide + radiusOfBallContainer;
	location2D.y = -(size.depth / 2) + (index / gridHeight) * (2 * radiusOfBallContainer + distanceToNext) + distanceToSide + radiusOfBallContainer;
	location2D.rotate(angle);
	point3f location3D(location2D.x, location2D.y, bottomThickness);
	location3D += data[index]->getGripPoint();

	return location3D;
}
}
