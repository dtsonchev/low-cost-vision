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
// License: newBSD 
//  
// Copyright © 2012, HU University of Applied Sciences Utrecht. 
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
// - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
	bool moving,
	datatypes::size3f size,
	size_t gridWidth,
	size_t gridHeight,
	double distanceToSide,
	double distanceToNext,
	double radiusOfBallContainer,
	double bottomThickness) :
		Crate(name, position, angle, moving, size, crateContent) {

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

datatypes::point3f GridCrate::getContentGripLocation(size_t index) const {
	if (data.at(index) == NULL) {
		throw cratedemo::LocationIsEmptyException();
	}

	std::cout << "data.at(index)->getGripPoint().z = " << data.at(index)->getGripPoint().z << std::endl;
	return getContainerLocation(index) + data.at(index)->getGripPoint();
}
}
