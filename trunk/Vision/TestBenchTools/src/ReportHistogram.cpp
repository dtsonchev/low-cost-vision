//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        TestBenchTools
// File:           ReportHistogram.cpp
// Description:    This class is a ReportField containing a histogram representation of a list of int's or doubles.
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
#include <string>
#include <sstream>
#include <report/ReportHistogram.hpp>

void report::ReportHistogram::setColumnNames(const char* first, ...){
	using std::string;
	using std::vector;

	va_list ap;
	va_start(ap, first);

	if(ReportField::columnNames.empty()){
		ReportField::columnNames.push_back(first);
		ReportField::columnNames.push_back(va_arg(ap, const char*));
	} else {
		ReportField::columnNames.at(0) = first;
		ReportField::columnNames.at(1) = va_arg(ap, const char*);
	}
	va_end(ap);
}

std::string report::ReportHistogram::toString() {
	std::stringstream ss;

	for (unsigned int i = 0; i < binList.size() - 1; i++) {
		ss << "From " << (i * (range / bins)) << " - "
				<< ((i + 1) * (range / bins));
		ss << ";";
		ss << binList.at(i) << std::endl;
	}

	ss << "From " << range << ";";
	ss << binList.at(binList.size() - 1) << std::endl;

	return ss.str();
}
