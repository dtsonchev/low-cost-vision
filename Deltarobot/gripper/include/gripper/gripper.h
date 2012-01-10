//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        Gripper
// File:           gripper.h
// Description:    Library for controlling the gripper
// Author:         Kasper van nieuwland & Zep Mouris
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of Gripper.
//
// Gripper is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Gripper is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with Gripper.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************
#pragma once

#include <modbus/modbus.h>
#include <string>

/**
 * Software interface for gripper.
 */
class gripper
{
public:
	/**
	 * Constructor
	 * @param ip_adress the ip-adress of the IO unit.
	 * @param port the port number of the IO unit.
	 */
	gripper(const std::string & ip_adress, const int port);
	/**
	 * Destructor
	 */
	~gripper();

	/**
	 * Connects to the IO unit
	 */
	void connect();
	/**
	 * Disconnects to the IO unit
	 */
	void disconnect();

	/**
	 * Switches the gripper on
	 */
	void grab();
	/**
	 * Releases the gripper
	 */
	void release();

private:
	std::string ip_adress;
	modbus_t* modbus;
	const static int GRIPPER_MODBUS_ADRESS = 8001;
	int port;
	bool connected;
};
