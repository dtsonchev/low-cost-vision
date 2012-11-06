//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        TestBenchTools
// File:           Tools.hpp
// Description:    Functions to get meta data from an XML file and store
//                 it in an ImageMetaData object.
// Author:         Franc Pape
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

#pragma once

#include <vector>
#include <map>
#include <set>
#include <imageMetaData/Types.hpp>

namespace imageMetaData {

//========================================================================
// Functions for obtaining and manipulating image metadata
//========================================================================
/**
 * Extracts the meta data of images from an XML file and puts it in ImageMD objects
 * @param xmlFile the path of the XML file containing the meta data
 * @return a vector of ImageMD objects with the extracted meta data
 */
std::vector<ImageMD> getMetaData(std::string xmlFile);

/**
 * Parses the contents of a string and puts it in a AnyType as either an int, a double or a string
 * @param str the string to be interpreted
 * @return an AnyType object containing the parsed information
 */
AnyType AnyTypeFromString(std::string str);

/**
 * Checks if a map contains a certain key
 * @param m the map to be checked
 * @param k the key to be checked
 * @return true if it contains the key, false otherwise
 */
template <class K, class V>
bool ContainsKey(const std::map<K, V>& m, const K& k){
	return m.find(k) != m.end();
}

template <class K, class V>
std::vector<V> getAllValues(std::map<K, V>& m){
	std::vector<V> values;
	for(typename std::map<K, V>::iterator it = m.begin(); it != m.end(); ++it) {
		values.push_back(it->second);
	}
	return values;
}

}
