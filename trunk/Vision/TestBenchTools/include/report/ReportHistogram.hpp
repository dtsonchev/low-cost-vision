#pragma once
#include <vector>
#include <iostream>
#include <sstream>
#include <report/ReportField.hpp>

namespace report {
/**
 * This class is a ReportField containing a histogram representation
 * of a list of int's or doubles
 */
class ReportHistogram : public ReportField {
public:
	/**
	 * Creates a histogram from vector<T> vec, with bns bins and a range from 0 to rnge
	 */
	/**
	 * Creates a histogram from a list of values
	 * @param name the name of the field
	 * @param vec a list of int's ot doubles
	 * @param bns the number of ranges
	 * @param rnge the maximum value
	 */
	template<class T>
	ReportHistogram(const char* name, const std::vector<T>& vec, double bns, double rnge);

	/**
	 * Set the names of the columns
	 * @param first first the name of the first column
	 */
	virtual void setColumnNames(const char* first, ...);
	/**
	 * Returns a string representation of the histogram,
	 * with the columns seperated by ';' and each row on a new line
	 * @return the formatted string
	 */
	virtual std::string toString();

private:
	double bins;
	double range;
	std::vector<int> binList;
};

}

template<class T>
report::ReportHistogram::ReportHistogram(const char* name, const std::vector<T>& vec, double bns, double rnge) :
ReportField(name)
{
	bins = bns;
	range = rnge;

	for (unsigned int i = 0; i < bins; ++i) {
		binList.push_back(0);
	}

	binList.push_back(0); // Add recycle bin for out of range values

	for (unsigned int it = 0; it < vec.size(); it++) {
		T val = vec.at(it);
		if(val > range) // If value is out of range
			binList.at(binList.size() - 1)++;
		else
			binList.at((val / (range/bins)))++ ;
	}
}
