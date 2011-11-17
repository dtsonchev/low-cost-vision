#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <iostream>
#include "Cell.hpp"
#include "cellActions.hpp"
#include "ReportField.hpp"
#include "ReportList.hpp"

ReportList::ReportList(std::vector<Type> col) {
	hasSumRow =false;
	hasAverageRow =false;
	hasPercentRow =false;
	using std::vector;
	columns = col;
	numberOfColumns = columns.size();
	for (uint i = 0; i < col.size(); i++) {
		vector<Cell *> row;
		data.push_back(row);
	}
}

void ReportList::appendRow(const int * cell1, ...) {
	using std::string;
	using std::vector;
	data.at(0).push_back(report::newCell(*(int*)(cell1)));
	va_list ap;
	va_start(ap, cell1);
	uint cellCounter = 1;
	void * param = va_arg(ap,void *);
	while(!(param == NULL || cellCounter >= columns.size())) {
		switch(columns.at(cellCounter)) {
			case INT:
			data.at(cellCounter).push_back(report::newCell(*(int*)(param)));
			break;
			case DOUBLE:
			data.at(cellCounter).push_back(report::newCell(*(double*)(param)));
			break;
			case STRING:
			data.at(cellCounter).push_back(report::newCell(*(char *)(param)));
			break;
		}
		cellCounter++;
		param = va_arg(ap,void *);
	}
	va_end(ap);
}
void ReportList::appendRow(const double * cell1, ...) {
	using std::string;
	using std::vector;
	data.at(0).push_back(report::newCell(*(double*)(cell1)));
	va_list ap;
	va_start(ap, cell1);
	uint cellCounter = 1;
	void * param = va_arg(ap,void *);
	while(!(param == NULL || cellCounter >= columns.size())) {
		switch(columns.at(cellCounter)) {
			case INT:
			data.at(cellCounter).push_back(report::newCell(*(int*)(param)));
			break;
			case DOUBLE:
			data.at(cellCounter).push_back(report::newCell(*(double*)(param)));
			break;
			case STRING:
			data.at(cellCounter).push_back(report::newCell(*(char *)(param)));
			break;
		}
		cellCounter++;
		param = va_arg(ap,void *);
	}
	va_end(ap);
}

void ReportList::appendRow(const char * cell1, ...) {
	using std::string;
	using std::vector;
	data.at(0).push_back(report::newCell(cell1));
	va_list ap;
	va_start(ap, cell1);
	uint cellCounter = 1;
	void * param = va_arg(ap,void *);
	while(!(param == NULL || cellCounter >= columns.size())) {
		switch(columns.at(cellCounter)) {
			case INT:
			appendCell(cellCounter,*(int*)(param));
			break;
			case DOUBLE:
			appendCell(cellCounter,*(double*)(param));
			break;
			case STRING:
			appendCell(cellCounter,*(char*)(param));
			break;
		}
		cellCounter++;
		param = va_arg(ap,void *);
	}
	va_end(ap);
}

void ReportList::enableSumRow(bool sum) {
	hasSumRow = sum;
}

void ReportList::enableAverageRow(bool avg) {
	hasAverageRow = avg;
}

void ReportList::enablePercentRow(bool perc) {
	hasPercentRow = perc;
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

std::string ReportList::toString() {
	std::stringstream ss;
	for (uint row = 0; row < data.at(0).size(); row++) {
		for (uint col = 0; col < data.size(); col++) {
			ss << data.at(col).at(row)->toString() << "\t";
		}

		ss << std::endl;

	}
	if(hasSumRow){
		appendSumRowToSs(ss);
	}
	if(hasAverageRow){
		appendAverageRowToSs(ss);
	}
	if(hasPercentRow){
		//TODO: this
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
