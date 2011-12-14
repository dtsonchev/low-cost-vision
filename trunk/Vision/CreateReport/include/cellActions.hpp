//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        CreateReport
// File:           cellActions.hpp
// Description:    Functions to create an IntCell*, DoubleCell* or StringCell*.
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

#include <subCells.hpp>

namespace report {
	using std::string;
	static IntegerCell * newCell(int val) { return new IntegerCell(val); }
	static DoubleCell  * newCell(double val) { return new DoubleCell(val); }
	static StringCell  * newCell(const char * val) { return new StringCell(val); }
}
