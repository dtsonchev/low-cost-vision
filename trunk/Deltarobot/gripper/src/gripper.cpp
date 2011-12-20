//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        Gripper
// File:           gripper.cpp
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

#include <gripper/gripper.h>
#include <cerrno>
#include <stdexcept>
#include <string>
#include <sstream>

gripper::gripper(const std::string & ip_adress, const int port) :
	ip_adress(ip_adress),
	port(port),
	connected(false)
{
}

gripper::~gripper()
{
	try
	{
		if(connected)
		{
			disconnect();
		}
	}
	catch(...)
	{
	}
}

void gripper::connect()
{
	if(!connected)
	{
		modbus = modbus_new_tcp(ip_adress.c_str(), port);
		if(modbus == NULL)
		{
			throw std::runtime_error("Unable to allocate libmodbus context");
		}
		if(modbus_connect(modbus) == -1)
		{
			throw std::runtime_error("Connection failed");
		}
		connected = true;
	}
	else
	{
		throw std::runtime_error("Already connected");
	}
}

void gripper::disconnect()
{
	if(connected)
	{
		modbus_close(modbus);
		modbus_free(modbus);
		connected = false;
	}
	else
	{
		throw std::runtime_error("Already disconnected");
	}
}

void gripper::grap()
{
	if(!connected)
	{
		throw std::runtime_error("Could not grab: disconnected");
	}

	if(modbus_write_register(modbus, GRIPPER_MODBUS_ADRESS, 1) == -1)
	{
		std::stringstream ss;
		ss << "Could not grab: modbus error: ";
		ss << modbus_strerror(errno);
		throw std::runtime_error(ss.str());
	}
}

void gripper::release()
{
	if(!connected)
	{
		throw std::runtime_error("Could not release: disconnected");
	}

	if(modbus_write_register(modbus, GRIPPER_MODBUS_ADRESS, 0) == -1)
	{
		std::stringstream ss;
		ss << "Could not release: modbus error: ";
		ss << modbus_strerror(errno);
		throw std::runtime_error(ss.str());
	}
}

