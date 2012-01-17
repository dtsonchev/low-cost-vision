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
#include <cratedemo/CrateDemo.hpp>
#include <cratedemo/GridCrate.hpp>
#include <cratedemo/Crate.hpp>
#include <cratedemo/CrateExceptions.hpp>
#include <datatypes/point2.hpp>
#include <datatypes/point3.hpp>
#include <datatypes/size3.hpp>
#include <string>
#include <iostream>
namespace cratedemo {

GridCrate::GridCrate(
	std::string name,
	std::vector<CrateContent*>& crateContent,
	datatypes::point2f position,
	float angle,
	datatypes::size3f size,
	size_t gridWidth,
	size_t gridHeight,
	double distanceToSide,
	double distanceToNext,
	double radiusOfBallContainer,
	double bottomThickness) :
		Crate(name, position, angle, size, crateContent) {

	this->gridWidth = gridWidth;
	this->gridHeight = gridHeight;
	this->distanceToSide = distanceToSide;
	this->distanceToNext = distanceToNext;
	this->bottomThickness = bottomThickness;
	this->radiusOfBallContainer = radiusOfBallContainer;
}

datatypes::point3f GridCrate::getContainerLocation(size_t index) const {
	using namespace datatypes;

	point2f location2D;
	location2D.x = (-size.width / 2.0) + (index % gridWidth) * (2.0 * radiusOfBallContainer + distanceToNext) + distanceToSide + radiusOfBallContainer;
	location2D.y = (-size.depth / 2.0) + (index / gridHeight) * (2.0 * radiusOfBallContainer + distanceToNext) + distanceToSide + radiusOfBallContainer;
	location2D = location2D.rotate(-angle);
	location2D += position;
	std::cout << "TABLE_HEIGHT = " << TABLE_HEIGHT << " bottomThickness = " << bottomThickness << " (TABLE_HEIGHT+bottomThickness) = " << (TABLE_HEIGHT+bottomThickness) << std::endl;
	point3f location3D(location2D.x, location2D.y, (TABLE_HEIGHT+bottomThickness));
	//std::cout << "getContainerLocation:\nx:\t" << location3D.x << "\ny:\t" << location3D.y << "\nz:\t" << location3D.z << std::endl;
	return location3D;
}

datatypes::point3f GridCrate::getCrateContentGripLocation(size_t index) const {
	if (data.at(index) == NULL) {
		throw cratedemo::LocationIsEmptyException();
	}

	std::cout << "data.at(index)->getGripPoint().z = " << data.at(index)->getGripPoint().z << std::endl;
	return getContainerLocation(index) + data.at(index)->getGripPoint();
}
}
