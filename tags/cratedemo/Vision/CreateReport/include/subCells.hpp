//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        CreateReport
// File:           subCells.hpp
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
#include <sstream>
#include <Cell.hpp>


class IntegerCell : public Cell {
public:
	IntegerCell(int val) : Cell(INT), value(val) {}
	int getValue() { return value;}
	std::string toString() {
		std::stringstream ss;
		ss << value;
		return ss.str();
	}
private:
	int value;
};

class DoubleCell : public Cell {
public:
	DoubleCell(double val) : Cell(DOUBLE), value(val) {}
	double getValue() { return value;}
	virtual ~DoubleCell(){};
	std::string toString() { std::stringstream ss; ss << value; return ss.str(); }
private:
	double value;
};

class StringCell : public Cell {
public:
	StringCell(std::string val) : Cell(STRING), value(val) {}
	virtual ~StringCell(){}
	std::string getValue() { return value;}
	std::string toString() { return value; }
private:
	std::string value;
};

//TODO:Telsell

