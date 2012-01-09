//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        cratedemo
// File:           MiniBall.hpp
// Description:    Representation of a small ball.
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
#pragma once
#include <datatypes/point3.hpp>
#include <datatypes/size3.hpp>
#include <datatypes/Color.hpp>
#include <cratedemo/CrateContent.hpp>
namespace cratedemo {
namespace Color {
	typedef enum _Color
	{
		BLACK,
		RED,
		ORANGE,
		YELLOW,
		GREEN,
		BLUE,
		INDIGO,
		VIOLET,
		WHITE
	} type;
}

class MiniBall : public CrateContent {
private:
	Color::type color;
	static const float SIZE_WIDTH = 9.6;
	static const float SIZE_DEPTH = 9.6;
	static const float SIZE_HEIGHT = 9.6;
	static const float GRIPPOINT_X = 0.0;
	static const float GRIPPOINT_Y = 0.0;
	static const float GRIPPOINT_Z = 9.6;
	static const double WEIGHT = 4.0;
public:
	MiniBall(Color::type color) :
		CrateContent(datatypes::point3f(GRIPPOINT_X,GRIPPOINT_Y,GRIPPOINT_Z), datatypes::size3f(SIZE_WIDTH,SIZE_DEPTH,SIZE_HEIGHT), WEIGHT){
		this->color = color;
	}
	Color::type getColor() const { return color;}
	~MiniBall() {}
};
}
