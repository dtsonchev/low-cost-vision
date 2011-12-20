//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        TestBenchTools
// File:           CategoryOverview.cpp
// Description:    This class is a ReportField that represents the results of a certain category.
// Author:         Wouter Langerak
// Notes:          
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

#include <stdarg.h>
#include <vector>
#include <sstream>
#include <boost/foreach.hpp>
#include <report/CategoryOverview.hpp>
#include <report/ReportField.hpp>
#include <report/ReportList.hpp>

#ifdef __CDT_PARSER__
    #define foreach(a, b) for(a : b)
#else
    #define foreach(a, b) BOOST_FOREACH(a, b)
#endif

report::CategoryOverview::CategoryOverview(const imageMetaData::CategoryResults& cr) :
		ReportList(cr.first.c_str(), 3, STRING, INT, STRING) {
	std::stringstream percent;
	foreach(imageMetaData::SubCategoryResults scr, cr.second){
		percent.str("");
		percent << ((scr.second.first *100 )/ scr.second.second) << "%";
		appendRow(scr.first.c_str(), scr.second.first,percent.str().c_str());
	}

}
