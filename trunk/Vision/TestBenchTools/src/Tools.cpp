//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        TestBenchTools
// File:           Tools.cpp
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

#include <iostream>
#include <sstream>
#include <map>
#include <vector>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <imageMetaData/Types.hpp>
#include <imageMetaData/Tools.hpp>

#ifdef __CDT_PARSER__
#define foreach(a, b) for(a : b)
#else
#define foreach(a, b) BOOST_FOREACH(a, b)
#endif

std::vector<imageMetaData::ImageMD> imageMetaData::getMetaData(std::string path,
		std::string rootTag) {
	using std::string;
	using std::stringstream;
	using std::vector;
	using std::map;
	using boost::property_tree::ptree;

	vector<ImageMD> md;
	ptree pt;

	read_xml(path, pt);
	foreach(ptree::value_type &img, pt.get_child(rootTag)){
		ImageMD imd(img.second.get<string>("<xmlattr>.path"));

		foreach(ptree::value_type &img_child, img.second) {
			if(img_child.first == "category") {
				imd.categories[img_child.second.get<string>("<xmlattr>.name")] =
						img_child.second.get<string>("<xmlattr>.value");
			} else if(img_child.first == "property") {
				imd.properties[img_child.second.get<string>("<xmlattr>.name")] =
						AnyTypeFromString(img_child.second.get<string>("<xmlattr>.value"));
			} else if(img_child.first == "object") {
				map<string, AnyType> properties;

				foreach(ptree::value_type &obj_child, img_child.second) {
					if(obj_child.first == "property") {
						properties[obj_child.second.get<string>("<xmlattr>.name")] =
								AnyTypeFromString(obj_child.second.get<string>("<xmlattr>.value"));
					}
				}

				imd.objects.push_back(properties);
			}
		}

		md.push_back(imd);
	}

	return md;
}

imageMetaData::AnyType imageMetaData::AnyTypeFromString(std::string str){
	using std::stringstream;

	stringstream ss;
	ss << str;

	// If string contains no decimal point
	if(str.find_first_of('.') == std::string::npos){
		// Check for integer
		int i;
		ss >> i;
		if(!ss.fail()){
			return AnyType(i);
		}
	} else {
		// Check for double
		double d;
		ss >> d;
		if(!ss.fail()){
			return AnyType(d);
		}
	}

	// Otherwise it must be a string
	return AnyType(ss.str());
}
