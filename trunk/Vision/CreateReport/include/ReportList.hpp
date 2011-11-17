#pragma once

#include <string>
#include <vector>
#include "ReportField.hpp"
#include "Cell.hpp"

class ReportList : public ReportField {

public:
	ReportList();
	ReportList(std::vector<Type> col);
	void appendRow(const int * cell1, ...);
	void appendRow(const double * cell1, ...);
	void appendRow(const char * cell1, ...);
	void enableSumRow(bool sum);
	void enableAverageRow(bool avg);
	void enablePercentRow(bool perc);
	std::string toString();
private:
	bool hasSumRow;
	bool hasAverageRow;
	bool hasPercentRow;
	int numberOfColumns;
	std::vector<Type> columns;
	std::vector< std::vector<Cell*> > data;
	void appendCell(const int column, const int value);
	void appendCell(const int column,const double value);
	void appendCell(const int column,const char * value);
	void appendCell(const int column, std::string value);
	void appendSumRowToSs(std::stringstream& ss);
	void appendAverageRowToSs(std::stringstream& ss);

};

