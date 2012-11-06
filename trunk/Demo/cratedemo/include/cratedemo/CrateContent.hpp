//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        cratedemo
// File:           CrateContent.hpp
// Description:    Representation of an object which can be placed in a crate.
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
#include <string>
#include <map>
#include <vector>
/*
gripPoint is the offset of the most left point of the crateContent.
For example in the picture(of an object seen from above) the gripPoint(*) has an offset of 7,0,-4.
0,0,0 ---------------X
|██████|██████
| █████|█████
|  ████|████
|------*████
|  █████████
|    █████
|     ███
|      █
Z
 */
/**
 * Representation of an object which can be placed in a crate.
 */

namespace cratedemo {

class CrateContent {

public:
	/**
	 * Returns the gripPoint
	 * @return
	 */
	datatypes::point3f getGripPoint() const { return gripPoint; }
	/**
	 * Returns the size
	 * @return
	 */
	datatypes::size3f getSize() const { return size; }
	/**
	 * Returns the size.
	 * @return
	 */
	double getWeight() const { return weight; }

protected:
	/**
	 *
	 * @param gripPoint gripPoint is the offset of the most left point of the crateContent.
	 * @param size size of the object
	 * @param weight weight of the object
	 */
	CrateContent(
		datatypes::point3f gripPoint,
		datatypes::size3f size,
		double weight) :
			gripPoint(gripPoint),
			size(size),weight(weight) {}

private:
	datatypes::point3f gripPoint; //in mm
	datatypes::size3f size; //in mm
	float weight; //in gram
};

typedef std::map<std::string, std::vector<CrateContent*> > CrateContentMap;

}
