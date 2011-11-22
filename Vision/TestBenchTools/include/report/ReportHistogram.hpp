#pragma once
#include <vector>
#include <iostream>
#include <sstream>
#include <report/ReportField.hpp>

namespace report {
/**
 * Class which can make a histogram of a given vector<int> vector<double>
 */
class ReportHistogram : public ReportField {
public:
	/**
	 * Creates a histogram from vector<T> vec, with bns bins and a range from 0 to rnge
	 */
	template<class T>
	ReportHistogram(const char* name, const std::vector<T>& vec, double bns, double rnge);

	virtual void setColumnNames(const char* first, ...);
	/**
	 * returns a std::string representation of the histogram
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
	for (unsigned int it = 0; it < vec.size(); it++) {
		binList.at((vec.at(it) / (range/bins)))++ ;
	}
}
