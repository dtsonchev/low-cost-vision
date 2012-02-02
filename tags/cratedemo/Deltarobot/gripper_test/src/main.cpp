//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        Gripper_test
// File:           main.cpp
// Description:    Test main for controlling the gripper
// Author:         Kasper van nieuwland & Zep Mouris
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of Gripper_test.
//
// Gripper_test is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Gripper_test is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with Gripper_test.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************
#include <iostream>
#include <gripper/gripper.h>

using namespace std;

int main(void)
{
	gripper grip("192.168.0.2", 502);
	grip.connect();
	int input;
	do {
		std::cin >> input;
		if(input == 0)
		{
			grip.grab();
		} 
		else if(input == 1)
		{
			grip.release();
		}

	} while(input != 2);

	grip.disconnect();
	return 0;
}


