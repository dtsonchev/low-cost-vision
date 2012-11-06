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
