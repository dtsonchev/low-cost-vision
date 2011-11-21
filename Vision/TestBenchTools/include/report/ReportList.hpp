#pragma once
#include <string>
#include <vector>
#include <report/ReportField.hpp>
#include <report/Cell.hpp>

namespace report {

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

}
