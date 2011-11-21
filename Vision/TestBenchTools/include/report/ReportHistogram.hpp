#pragma once
#include <vector>
#include <iostream>
#include <sstream>
#include <report/ReportField.hpp>

namespace report {

class ReportHistogram : public ReportField {
public:
	template<class T>
	ReportHistogram(const std::vector<T>& vec, unsigned int bns, unsigned int rnge);
	std::string toString();

private:
	unsigned int bins;
	unsigned int range;
	std::vector<int> binList;

};

}

template<class T>
report::ReportHistogram::ReportHistogram(const std::vector<T>& vec, unsigned int bns, unsigned int rnge) {
	bins = bns;
	range = rnge;

	for (unsigned int i = 0; i < bins; ++i) {
		binList.push_back(0);
	}
	for (unsigned int it = 0; it < vec.size(); it++) {
		std::cout << ((vec.at(it) / (range/bins)))+1 << std::endl;
		binList.at((vec.at(it) / (range/bins)))++ ;
	}
}
