#pragma once
#include <string>
#include <vector>
#include <report/ReportField.hpp>
#include <report/Cell.hpp>

namespace report {
/**
 * Class which can contain a list.
 */
class ReportList: public ReportField {

public:
	/**
	 * Create an empty reportlist
	 */
	ReportList();
	/**
	 * Create an reportlist which contains columns with the types that are given in the vector<Type> col.
	 * Available types are INT, DOUBLE and STRING.
	 */
	ReportList(std::vector<Type> col);
	/**
	 * Append a new row to the reportlist
	 */
	void appendRow(const int cell1, ...);
	/**
	 * Append a new row to the reportlist
	 */
	void appendRow(const double cell1, ...);
	/**
		 * Append a new row to the reportlist
		 */
	void appendRow(const char * cell1, ...);
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
