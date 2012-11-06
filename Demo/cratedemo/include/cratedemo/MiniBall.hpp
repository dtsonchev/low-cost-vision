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
/**
 * Mini balls used in for the demo
 */
class MiniBall : public CrateContent {
private:
	Color::type color;
	static const float SIZE_WIDTH = 9.7;
	static const float SIZE_DEPTH = 9.7;
	static const float SIZE_HEIGHT = 9.7;
	static const float GRIPPOINT_X = 0.0;
	static const float GRIPPOINT_Y = 0.0;
	static const float GRIPPOINT_Z = 9.7;
	static const double WEIGHT = 0.4;

public:
	MiniBall(Color::type color) :
		CrateContent(
			datatypes::point3f(GRIPPOINT_X,GRIPPOINT_Y,GRIPPOINT_Z),
			datatypes::size3f(SIZE_WIDTH,SIZE_DEPTH,SIZE_HEIGHT),
			WEIGHT) {
		this->color = color;
	}

	~MiniBall() {}

	/**
	 * @return color of the miniball
	 */
	Color::type getColor() const { return color;}
};
}
