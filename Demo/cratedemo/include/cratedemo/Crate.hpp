//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        cratedemo
// File:           Crate.hpp
// Description:    Base class for crates.
// Author:         Wouter Langerak & Lukas Vermond
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

#include <cstdlib>
#include <string>

#include <cratedemo/CrateContent.hpp>
#include <datatypes/point2.hpp>
#include <datatypes/point3.hpp>
#include <datatypes/size3.hpp>

namespace cratedemo {
/**
 * Base class for crates.
 */
class Crate {

public:
	virtual ~Crate();
	/**
	 * Puts a CrateContent* in the list of containing items on location index.
	 * @param index Location of the to be inputed CrateContent*.
	 * @param crateContent Pointer of the to be inputed CrateContent.
	 */
	void put(size_t index, CrateContent* crateContent);
	/**
	 *
	 */
	CrateContent* get(size_t index) const;
	/**
	 *
	 */
	virtual datatypes::point3f getContainerLocation(size_t index) const = 0;
	/**
	 * Returns the location of the object on index.
	 * @param index
	 * @return the location of the object on index.
	 */
	virtual datatypes::point3f getContentGripLocation(size_t index) const = 0;
	/**
	 * Remove the content on location index.
	 * @param index
	 */
	void remove(size_t index);
	/**
	 * Returns if the Crate is empty.
	 * @return
	 */
	bool isEmpty() const;

	const std::string& getName(void) const;
protected:
	Crate(
		std::string name,
		datatypes::point2f position,
		float angle,
		bool moving,
		datatypes::size3f size,
		std::vector<CrateContent*>& crateContent);

	std::string name;
public:
	datatypes::point2f position;
	float angle;
	bool moving;
protected:
	datatypes::size3f size;
	std::vector<CrateContent*>& data;

};
}
