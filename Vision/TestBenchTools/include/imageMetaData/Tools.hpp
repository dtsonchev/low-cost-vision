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
// License:        GNU GPL v3
//
// This file is part of TestBenchTools.
//
// TestBenchTools is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// TestBenchTools is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with TestBenchTools.  If not, see <http://www.gnu.org/licenses/>.
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
 * @param path the path of the XML file containing the meta data
 * @param rootTag the first tag in the XML file
 * @return a vector of ImageMD objects with the extracted meta data
 */
std::vector<ImageMD> getMetaData(std::string path, std::string rootTag);

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
