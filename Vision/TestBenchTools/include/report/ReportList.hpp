#pragma once
#include <stdarg.h>
#include <string>
#include <vector>
#include <report/ReportField.hpp>
#include <report/Cell.hpp>
#include <report/CellActions.hpp>

namespace report {
/**
 * Class which can contain a list.
 */
class ReportList: public ReportField {
public:
	ReportList();
	/**
	 * Create an reportlist which contains columns with the types that are given in the vector<Type> col.
	 * Available types are INT, DOUBLE and STRING.
	 */
	ReportList(const char* name, std::vector<Type> col);
	ReportList(const char* name, int numCols, ...);

	/**
	 * Append a new row to the reportlist
	 */
	template <class T>
	void appendRow(T  cell1, ...);

	/**
	 * Enable an extra row with sum per column
	 * @param sum bool to enable/disable this row
	 */
	void enableSumRow(bool sum);
	/**
	 * Enable an extra row with the average per column
	 * @param avg bool to enable/disable this row
	 */
	void enableAverageRow(bool avg);
	/**
	 * Enable an extra row with the percent of partrow in fullRow
	 * @param perc bool to enable/disable this row
	 * @param partRow the column number of the part part of the percent
	 * @param fullRow the column number of the full part of the percent
	 */
	void enablePercentRow(bool perc, unsigned int partRow, unsigned int fullRow);

	/**
	 * Returns a std::string representation of the reportlist
	 * @return
	 */
	virtual std::string toString();

	virtual void setColumnNames(const char* first, ...);
	void vsetColumnNames(const char* first, va_list ap);

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
