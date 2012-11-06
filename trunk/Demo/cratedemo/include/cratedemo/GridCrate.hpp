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

#pragma once

#include <cratedemo/Crate.hpp>
#include <cratedemo/Environment.hpp>
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
		double bottomThickness);

	virtual datatypes::point3f getContainerLocation(size_t index) const;
	virtual datatypes::point3f getContentGripLocation(size_t index) const;

private:
	size_t gridWidth;
	size_t gridHeight;
	float distanceToSide;
	float distanceToNext;
	float bottomThickness;
	float radiusOfBallContainer;
};
}
