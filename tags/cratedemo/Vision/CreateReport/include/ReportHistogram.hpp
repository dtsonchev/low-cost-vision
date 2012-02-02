//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        CreateReport
// File:           ReportHistogram.hpp
// Description:    This class is a ReportField containing a histogram representation of a list of int's or doubles.
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

#pragma once
#include <vector>
#include <iostream>
#include <sstream>

class ReportHistogram : public ReportField {
public:

template<class T>
ReportHistogram(const std::vector<T>& vec, unsigned int bns, unsigned rnge) {
	bins = bns;
	range = rnge;

	for (unsigned int i = 0; i < bins; ++i) {
		binList.push_back(0);
	}
	for (unsigned int it = 0; it < vec.size(); it++) {
		std::cout << ((vec.at(it) / (range/bins)))+1 << std::endl;
		binList.at((vec.at(it) / (range/bins)))++ ;
	}

}

std::string toString() {
	std::stringstream ss;

	for (unsigned int i = 0; i < binList.size(); i++) {
		ss << "From " << (i * (range / bins)) << " - "
				<< ((i + 1) * (range / bins)) << ":\t" << binList.at(i)
				<< std::endl;
	}
	return ss.str();
}
private:
	unsigned int bins;
	unsigned int range;
	std::vector<int> binList;

};
