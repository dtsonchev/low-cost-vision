//******************************************************************************
// Project:        cratedemo
// File:           GridCrate4x4MiniBall.hpp
// Description:    Typical crate, 16 containers(4x4) for miniball
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
#include <cratedemo/GridCrate.hpp>

namespace cratedemo {
/**
 *	Typical crate, 16 containers(4x4) for miniball
 *
 */
class GridCrate4x4MiniBall : public GridCrate {
private:
	static const float CRATE_WIDTH;
	static const float CRATE_DEPTH;
	static const float CRATE_HEIGHT;
	static const int MAX_NUMBER_OF_OBJECTS = 16;
	static const int GRID_WIDTH = 4;
	static const int GRID_HEIGHT = 4;
	static const float DISTANCE_TO_SIDE;
	static const float DISTANCE_TO_NEXT;
	static const float RADIUS_OF_BALL_CONTAINER;
	static const float BOTTOM_THICKNESS;

public:
	/**
	 * Typical crate, 16 containers(4x4) for miniball
	 * @param name name of the crate
	 * @param position position of the crate
	 * @param angle rotation in the middle of the crate
	 */
	GridCrate4x4MiniBall(
			std::string name,
			datatypes::point2f position,
			float angle) : GridCrate(	name,
										position,
										angle,
										datatypes::size3f(CRATE_WIDTH,CRATE_DEPTH,CRATE_HEIGHT),
										MAX_NUMBER_OF_OBJECTS,
										GRID_WIDTH,
										GRID_HEIGHT,
										DISTANCE_TO_SIDE,
										DISTANCE_TO_NEXT,
										RADIUS_OF_BALL_CONTAINER,
										BOTTOM_THICKNESS) {}
};

}
