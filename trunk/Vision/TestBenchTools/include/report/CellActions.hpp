//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        TestBenchTools
// File:           cellActions.hpp
// Description:    Functions to create an IntCell*, DoubleCell* or StringCell*.
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

#pragma once
#include <report/SubCells.hpp>

namespace report {

	/**
	 * Creates a new IntegerCell with value
	 * @param val the value to put in the cell
	 * @return the new cell
	 */
	static IntegerCell* newCell(int val) { return new IntegerCell(val); }
	/**
	 * Creates a new DoubleCell with value
	 * @param val the value to put in the cell
	 * @return the new cell
	 */
	static DoubleCell* newCell(double val) { return new DoubleCell(val); }
	/**
	 * Creates a new StringCell with value
	 * @param val the value to put in the cell
	 * @return the new cell
	 */
	static StringCell* newCell(const char * val) { return new StringCell(val); }
	/**
	 * Creates a new StringCell with value
	 * @param val the value to put in the cell
	 * @return the new cell
	 */
	static StringCell* newCell(std::string& val) { return new StringCell(val.c_str()); }
}
