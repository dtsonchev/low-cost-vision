#pragma once
#include <vector>
#include <iostream>
#include <sstream>

class ReportHistogram : public ReportField {
public:

template<class T>
ReportHistogram(const std::vector<T>& vec, unsigned int bns, unsigned rnge) {
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

std::string toString() {
	std::stringstream ss;

	for (unsigned int i = 0; i < binList.size(); i++) {
		ss << "From " << (i * (range / bins)) << " - "
				<< ((i + 1) * (range / bins)) << ":\t" << binList.at(i)
				<< std::endl;
	}
	return ss.str();
}
private:
	unsigned int bins;
	unsigned int range;
	std::vector<int> binList;

};
