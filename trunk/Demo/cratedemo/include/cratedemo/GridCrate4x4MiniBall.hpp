//******************************************************************************
// Project:        cratedemo
// File:           GridCrate4x4MiniBall.hpp
// Description:    Typical crate, 16 containers(4x4) for miniball
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
#include <cratedemo/GridCrate.hpp>

namespace cratedemo {
/**
 *	Typical crate, 16 containers(4x4) for miniball
 *
 */
class GridCrate4x4MiniBall : public GridCrate {
private:
	static const float CRATE_WIDTH = 45.72;
	static const float CRATE_DEPTH = 45.72;
	static const float CRATE_HEIGHT = 16.5;
	static const float GRID_WIDTH = 4.0;
	static const float GRID_HEIGHT = 4.0;
	static const float DISTANCE_TO_SIDE = 1.11;
	static const float DISTANCE_TO_NEXT = 0.5;
	static const float RADIUS_OF_BALL_CONTAINER = 5.25;
	static const float BOTTOM_THICKNESS = 5.3;

public:
	/**
	 * Typical crate, 16 containers(4x4) for miniball
	 * @param name name of the crate
	 * @param position position of the crate
	 * @param angle rotation in the middle of the crate
	 */
	GridCrate4x4MiniBall(
		std::string name,
		std::vector<CrateContent*>& crateContent,
		datatypes::point2f position,
		float angle,
		bool moving) :
			GridCrate(
				name,
				crateContent,
				position,
				angle,
				moving,
				datatypes::size3f(CRATE_WIDTH,CRATE_DEPTH,CRATE_HEIGHT),
				GRID_WIDTH,
				GRID_HEIGHT,
				DISTANCE_TO_SIDE,
				DISTANCE_TO_NEXT,
				RADIUS_OF_BALL_CONTAINER,
				BOTTOM_THICKNESS) {}
};

}
