#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <iostream>
#include <report/Cell.hpp>
#include <report/CellActions.hpp>
#include <report/ReportField.hpp>
#include <report/ReportList.hpp>

report::ReportList::ReportList() {
	hasSumRow = false;
	hasAverageRow = false;
	hasPercentRow = false;
	percPartRow = -1;
	percFullRow = -1;
}

report::ReportList::ReportList(std::vector<Type> col) {
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

void report::ReportList::appendRow(const int cell1, ...) {
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

void report::ReportList::appendRow(const double cell1, ...) {
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

void report::ReportList::appendRow(const char * cell1, ...) {
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
