#include <string>
#include <sstream>
#include <report/ReportHistogram.hpp>

std::string report::ReportHistogram::toString() {
	std::stringstream ss;

	for (unsigned int i = 0; i < binList.size(); i++) {
		ss << "From " << (i * (range / bins)) << " - "
				<< ((i + 1) * (range / bins)) << ":\t" << binList.at(i)
				<< std::endl;
	}
	return ss.str();
}
