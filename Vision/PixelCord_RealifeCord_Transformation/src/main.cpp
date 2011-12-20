//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        PixelCord_RealifeCord_Transformation
// File:           main.cpp
// Description:    test for convert reallife coordinates to pixel coordinates and vice versa
// Author:         Kasper van Nieuwland & Zep Mouris
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of PixelCord_RealifeCord_Transformation.
//
// PixelCord_RealifeCord_Transformation is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// PixelCord_RealifeCord_Transformation is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with PixelCord_RealifeCord_Transformation.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************
include <pcrctransformation/pcrctransformer.hpp>
#include <pcrctransformation/point2f.hpp>
#include <iostream>

using namespace pcrctransformation;

int main(void)
{
	std::cout<<"start"<<std::endl;
	point2f::point2fvector rc;
	rc.push_back(point2f(0, 0));
	rc.push_back(point2f(0, 10));
	rc.push_back(point2f(10, 10));

	point2f::point2fvector pc;
	pc.push_back(point2f(100, 100));
	pc.push_back(point2f(100, 0));
	pc.push_back(point2f(0, 0));

	pc_rc_transformer p(rc, pc);
	point2f result = p.to_pc(point2f(1,1));
	point2f result2 = p.to_rc(result);
	std::cout << result.x << ", " << result.y << std::endl;
	std::cout << result2.x << ", " << result2.y << std::endl;
	return 0;
}
