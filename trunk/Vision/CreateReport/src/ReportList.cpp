//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        CreateReport
// File:           ReportList.cpp
// Description:    This class is the most basic ReportField containing a matrix of values.
// Author:         Wouter Langerak
// Notes:          
//
// License: newBSD 
//  
// Copyright © 2012, HU University of Applied Sciences Utrecht. 
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
// - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//******************************************************************************

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <iostream>
#include <Cell.hpp>
#include <cellActions.hpp>
#include <ReportField.hpp>
#include <ReportList.hpp>

ReportList::ReportList() {
	hasSumRow = false;
	hasAverageRow = false;
	hasPercentRow = false;
	percPartRow = -1;
	percFullRow = -1;
}

ReportList::ReportList(std::vector<Type> col) {
	hasSumRow = false;
	hasAverageRow = false;
	hasPercentRow = false;
	percPartRow = -1;
	percFullRow = -1;
	using std::vector;
	columns = col;
	numberOfColumns = columns.size();
	for (unsigned int i = 0; i < col.size(); i++) {
		vector<Cell *> row;
		data.push_back(row);
	}
}

void ReportList::appendRow(const int cell1, ...) {
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

void ReportList::appendRow(const double cell1, ...) {
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

void ReportList::appendRow(const char * cell1, ...) {
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

void ReportList::enablePercentRow(bool perc, unsigned int partRow, unsigned int fullRow) {
	if ((columns.at(partRow) == INT || columns.at(partRow) == DOUBLE)
			&& (columns.at(fullRow) == INT || columns.at(fullRow) == DOUBLE)) {
		hasPercentRow = true;
		percPartRow = partRow;
		percFullRow = fullRow;
	} else {
		hasPercentRow = false;
	}
}

void ReportList::enableSumRow(bool sum) {
	hasSumRow = sum;
}

void ReportList::enableAverageRow(bool avg) {
	hasAverageRow = avg;
}

void ReportList::appendSumRowToSs(std::stringstream& ss) {
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

void ReportList::appendAverageRowToSs(std::stringstream& ss) {
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
				ss << "Average:";
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

void ReportList::appendPercentRowToSs(std::stringstream& ss) {
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

std::string ReportList::toString() {
	std::stringstream ss;
	for (unsigned int row = 0; row < data.at(0).size(); row++) {
		for (unsigned int col = 0; col < data.size(); col++) {
			ss << data.at(col).at(row)->toString() << "\t";
		}

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

void ReportList::appendCell(const int column, const int value) {
	data.at(column).push_back(report::newCell(value));
}

void ReportList::appendCell(const int column, const double value) {
	data.at(column).push_back(report::newCell(value));
}
void ReportList::appendCell(const int column, const char * value) {
	data.at(column).push_back(report::newCell(value));
}
void ReportList::appendCell(const int column, std::string value) {
	data.at(column).push_back(report::newCell(value.c_str()));
}
