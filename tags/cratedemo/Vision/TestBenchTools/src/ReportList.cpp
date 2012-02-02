//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        TestBenchTools
// File:           ReportList.cpp
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

#include <stdarg.h>
#include <iostream>
#include <report/Cell.hpp>
#include <report/CellActions.hpp>
#include <report/ReportField.hpp>
#include <report/ReportList.hpp>

report::ReportList::ReportList(const char* name, std::vector<Type> col) :
		ReportField(name) {
	hasSumRow = false;
	hasAverageRow = false;
	hasPercentRow = false;
	percPartRow = -1;
	percFullRow = -1;

	columns = col;
	numberOfColumns = columns.size();
	for (unsigned int i = 0; i < col.size(); i++) {
		std::vector<Cell *> row;
		data.push_back(row);
	}
}

report::ReportList::ReportList(const char* name, int numCols, ...) :
		ReportField(name) {
	hasSumRow = false;
	hasAverageRow = false;
	hasPercentRow = false;
	percPartRow = -1;
	percFullRow = -1;

	va_list ap;
	va_start(ap, numCols);
	for(int i = 0; i < numCols; ++i){
		columns.push_back((Type)va_arg(ap, int));
	}
	numberOfColumns = columns.size();

	for (unsigned int i = 0; i < numberOfColumns; i++) {
		std::vector<Cell *> row;
		data.push_back(row);
	}
}

void report::ReportList::vsetColumnNames(const char* first, va_list ap){
	using std::string;
	using std::vector;

	if(ReportField::columnNames.empty()){
		ReportField::columnNames.push_back(first);
		for(unsigned int i = 1; i < columns.size(); ++i){
			const char* temp = va_arg(ap, const char*);
			ReportField::columnNames.push_back(temp);
		}
	} else {
		ReportField::columnNames.at(0) = first;
		for(unsigned int i = 1; i < columns.size(); ++i){
			ReportField::columnNames.at(i) = va_arg(ap, const char*);
		}
	}
}

void report::ReportList::setColumnNames(const char* first, ...){
	va_list ap;
	va_start(ap, first);
	vsetColumnNames(first, ap);
	va_end(ap);
}

void report::ReportList::enablePercentRow(bool perc, unsigned int partRow, unsigned int fullRow) {
	if ((columns.at(partRow) == INT || columns.at(partRow) == DOUBLE)
			&& (columns.at(fullRow) == INT || columns.at(fullRow) == DOUBLE)) {
		hasPercentRow = true;
		percPartRow = partRow;
		percFullRow = fullRow;
	} else {
		hasPercentRow = false;
	}
}

void report::ReportList::enableSumRow(bool sum) {
	hasSumRow = sum;
}

void report::ReportList::enableAverageRow(bool avg) {
	hasAverageRow = avg;
}

void report::ReportList::appendSumRowToSs(std::stringstream& ss) {
	int intSum;
	double doubleSum;
	for (std::vector<Type>::iterator col = columns.begin(); col < columns.end();
			++col) {
		intSum = 0;
		doubleSum = 0;
		if (*col == STRING) {
			if (col - columns.begin() > 0) {
				//appendCell(col - columns.begin(), "");
				ss << "";
			} else {
				//appendCell(col - columns.begin(), "Sum:");
				ss << "Sum:\t";
			}
		} else {
			for (std::vector<Cell *>::iterator it = data.at(
					col - columns.begin()).begin();
					it < data.at(col - columns.begin()).end(); ++it) {
				if (*col == INT) {
					intSum += ((IntegerCell *) (*it))->getValue();
				} else if (*col == DOUBLE) {
					doubleSum += ((DoubleCell *) (*it))->getValue();
				}
			}
			if (*col == INT) {
				//appendCell((col - columns.begin()), intSum);
				ss << intSum << "\t";
			} else if (*col == DOUBLE) {
				//appendCell((col - columns.begin()), doubleSum);
				ss << doubleSum << "\t";
			}
		}
	}
	ss << std::endl;
}

void report::ReportList::appendAverageRowToSs(std::stringstream& ss) {
	int intSum;
	double doubleSum;
	double numberOfElements;
	for (std::vector<Type>::iterator col = columns.begin(); col < columns.end();
			++col) {
		intSum = 0;
		doubleSum = 0;
		numberOfElements = 0;
		if (*col == STRING) {
			if (col - columns.begin() > 0) {
				//appendCell(col - columns.begin(), "");
				ss << "" << "\t";
			} else {
				//appendCell(col - columns.begin(), "Average:");
				ss << "Average:\t";
			}
		} else {
			for (std::vector<Cell *>::iterator it = data.at(
					col - columns.begin()).begin();
					it < data.at(col - columns.begin()).end(); ++it) {
				if (*col == INT) {
					++numberOfElements;
					intSum += ((IntegerCell *) (*it))->getValue();
				} else if (*col == DOUBLE) {
					++numberOfElements;
					doubleSum += ((DoubleCell *) (*it))->getValue();
				}
			}
			if (*col == INT) {
				//appendCell(col - columns.begin(),((int) (intSum / numberOfElements)));
				ss << ((int) (intSum / numberOfElements)) << "\t";
			} else if (*col == DOUBLE) {
				//appendCell((col - columns.begin()), doubleSum/numberOfElements);
				ss << (doubleSum / numberOfElements) << "\t";
			}
		}
	}
	ss << std::endl;
}

void report::ReportList::appendPercentRowToSs(std::stringstream& ss) {
	double partSum = 0;
	double fullSum = 0;
	for (std::vector<Cell *>::iterator it = data.at(percPartRow).begin();
			it < data.at(percPartRow).end(); ++it) {
		if (columns.at(percPartRow) == INT) {
			partSum += (double) (((IntegerCell *) (*it))->getValue());
		} else if (columns.at(percPartRow) == DOUBLE) {
			partSum += ((DoubleCell *) (*it))->getValue();
		}
	}
	for (std::vector<Cell *>::iterator it = data.at(percFullRow).begin();
			it < data.at(percFullRow).end(); ++it) {
		if (columns.at(percFullRow) == INT) {
			fullSum += (double) ((((IntegerCell *) (*it))->getValue()));
		} else if (columns.at(percFullRow) == DOUBLE) {
			fullSum += (((DoubleCell *) (*it))->getValue());
		}
	}
	for (unsigned int i = 0; i < columns.size(); ++i) {
		if (i != percFullRow) {
			ss << "\t";

		} else {
			ss << ((partSum * 100.0) / fullSum) << "%\t" << "(" << partSum
					<< "/" << fullSum << ")\t";
		}
	}
	ss << std::endl;
}

std::string report::ReportList::toString() {
	std::stringstream ss;
	for (unsigned int row = 0; row < data.at(0).size(); row++) {
		for (unsigned int col = 0; col < data.size() - 1; col++) {
			ss << data.at(col).at(row)->toString() << ";";
		}
		ss << data.at(data.size() - 1).at(row)->toString();

		ss << std::endl;

	}
	if (hasSumRow) {
		appendSumRowToSs(ss);
	}
	if (hasAverageRow) {
		appendAverageRowToSs(ss);
	}
	if (hasPercentRow) {
		appendPercentRowToSs(ss);
	}
	return ss.str();
}

void report::ReportList::appendCell(const int column, const int value) {
	data.at(column).push_back(report::newCell(value));
}
void report::ReportList::appendCell(const int column, const double value) {
	data.at(column).push_back(report::newCell(value));
}
void report::ReportList::appendCell(const int column, const char * value) {
	data.at(column).push_back(report::newCell(value));
}
void report::ReportList::appendCell(const int column, std::string value) {
	data.at(column).push_back(report::newCell(value.c_str()));
}
