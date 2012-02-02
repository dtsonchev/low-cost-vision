//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        VisionNode
// File:           main.cpp
// Description:    Starts the visionNode.
// Author:         Kasper van Nieuwland en Zep Mouris
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of VisionNode.
//
// VisionNode is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// VisionNode is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with VisionNode.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************
#include <vision/visionNode.h>

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "vision");

	visionNode vn(argc, argv);
	vn.run();

	return 0;
}
