//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        CreateReport
// File:           CategoryOverview.cpp
// Description:    This class is a ReportField that represents the results of a certain category.
// Author:         Wouter Langerak
// Notes:          
//
// License:        GNU GPL v3
//
// This file is part of CreateReport.
//
// CreateReport is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// CreateReport is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with CreateReport.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#include <vector>
#include <CategoryOverview.hpp>
#include <ReportField.hpp>
#include <ReportList.hpp>
#include <boost/foreach.hpp>

#include <sstream>
#ifdef __CDT_PARSER__
    #define foreach(a, b) for(a : b)
#else
    #define foreach(a, b) BOOST_FOREACH(a, b)
#endif

CatergoryOverview::CatergoryOverview(const ImageMetaData::CategoryResults& cr) : ReportField(cr.first) {
	//typedef std::pair<std::string, std::map<std::string, std::pair<int, int> > > CategoryResults;
	name = cr.first;

	std::vector<Type> columns;
	columns.push_back(STRING);
	columns.push_back(INT);
	columns.push_back(STRING);
	reportlist = ReportList(columns);
	std::stringstream percent;
	foreach(ImageMetaData::SubCategoryResults scr, cr.second){
		percent.str("");
		percent << ((scr.second.first *100 )/ scr.second.second) << "%";
		reportlist.appendRow(scr.first.c_str(), scr.second.first,percent.str().c_str());
	}

}
std::string CatergoryOverview::toString() {
	std::stringstream ss;
	ss << name  << ":" << std::endl;
	ss << reportlist.toString();
	return ss.str();
}
