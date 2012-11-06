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

void gripper::grab()
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

