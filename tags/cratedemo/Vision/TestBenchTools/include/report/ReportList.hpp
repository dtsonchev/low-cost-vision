//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        TestBenchTools
// File:           ReportList.hpp
// Description:    This class is the most basic ReportField containing a matrix of values.
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
#include <stdarg.h>
#include <string>
#include <vector>
#include <report/ReportField.hpp>
#include <report/Cell.hpp>
#include <report/CellActions.hpp>

namespace report {

/**
 * This class is the most basic ReportField containing a matrix of values.
 */
class ReportList: public ReportField {
public:
	/**
	 * Create a reportlist that contains columns with the types that are given in the vector<Type> col.
	 * Available types are INT, DOUBLE and STRING.
	 * @param name the name of the field
	 * @param col
	 */
	ReportList(const char* name, std::vector<Type> col);
	/**
	 * Create a reportlist that contains columns with the types that are given.
	 * Available types are INT, DOUBLE and STRING.
	 * @param name the name of the field
	 * @param numCols the number of columns you want
	 */
	ReportList(const char* name, int numCols, ...);

	/**
	 * Append a new row to the reportlist
	 * @param cell1 the first value in the row
	 */
	template <class T>
	void appendRow(T  cell1, ...);

	/**
	 * Enable or disable an extra row with sum per column
	 * @param sum bool to enable/disable this row
	 */
	void enableSumRow(bool sum = true);
	/**
	 * Enable or disable an extra row with the average per column
	 * @param avg bool to enable/disable this row
	 */
	void enableAverageRow(bool avg = true);
	/**
	 * Enable an extra row with the percent of partrow in fullRow
	 * @param perc bool to enable/disable this row
	 * @param partRow the column number of the part part of the percent
	 * @param fullRow the column number of the full part of the percent
	 */
	void enablePercentRow(bool perc, unsigned int partRow, unsigned int fullRow);

	/**
	 * Returns a string representation of the reportlist,
	 * with the columns seperated by ';' and each row on a new line
	 * @return the formatted string
	 */
	virtual std::string toString();

	/**
	 * Set the names of the columns
	 * @param first the name of the first column
	 */
	virtual void setColumnNames(const char* first, ...);
	/**
	 * Set the names of the columns (with va_list)
	 * @param first the name of the first column
	 * @param ap list of parameters containing the names of the rest of the columns
	 */
	void vsetColumnNames(const char* first, va_list ap);

	/**
	 * Destructor
	 */
	virtual ~ReportList(){}

protected:
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

}

template <class T>
void report::ReportList::appendRow(T cell1, ...) {
	using std::string;
	using std::vector;
	data.at(0).push_back(report::newCell(cell1));
	va_list ap;
	va_start(ap, cell1);
	unsigned int cellCounter = 1;
	while(!(cellCounter >= columns.size())) {
		switch(columns.at(cellCounter)) {
			case INT:
			appendCell(cellCounter,va_arg(ap,int));
			break;
			case DOUBLE:
			appendCell(cellCounter,va_arg(ap,double));
			break;
			case STRING:
			appendCell(cellCounter,va_arg(ap,char *));
			break;
		}
		cellCounter++;
	}
	va_end(ap);
}
