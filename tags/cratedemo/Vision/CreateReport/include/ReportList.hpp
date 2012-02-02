//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        CreateReport
// File:           ReportList.hpp
// Description:    This class is the most basic ReportField containing a matrix of values.
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

#include <string>
#include <vector>
#include <ReportField.hpp>
#include <Cell.hpp>

class ReportList : public ReportField {

public:
	ReportList();
	ReportList(std::vector<Type> col);
	void appendRow(const int  cell1, ...);
	void appendRow(const double cell1, ...);
	void appendRow(const char * cell1, ...);
	void enableSumRow(bool sum);
	void enableAverageRow(bool avg);
	void enablePercentRow(bool perc, unsigned int partRow, unsigned int fullRow);
	std::string toString();
private:
	bool hasSumRow;
	bool hasAverageRow;
	bool hasPercentRow;
	unsigned int numberOfColumns;
	unsigned int percPartRow;
	unsigned int percFullRow;
	std::vector<Type> columns;
	std::vector< std::vector<Cell*> > data;
	void appendCell(const int column, const int value);
	void appendCell(const int column,const double value);
	void appendCell(const int column,const char * value);
	void appendCell(const int column, std::string value);
	void appendSumRowToSs(std::stringstream& ss);
	void appendAverageRowToSs(std::stringstream& ss);
	void appendPercentRowToSs(std::stringstream& ss);

};

