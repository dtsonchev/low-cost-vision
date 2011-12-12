#include <stdarg.h>
#include <string>
#include <sstream>
#include <report/ReportHistogram.hpp>

void report::ReportHistogram::setColumnNames(const char* first, ...){
	using std::string;
	using std::vector;

	va_list ap;
	va_start(ap, first);

	if(ReportField::columnNames.empty()){
		ReportField::columnNames.push_back(first);
		ReportField::columnNames.push_back(va_arg(ap, const char*));
	} else {
		ReportField::columnNames.at(0) = first;
		ReportField::columnNames.at(1) = va_arg(ap, const char*);
	}
	va_end(ap);
}

std::string report::ReportHistogram::toString() {
	std::stringstream ss;

	for (unsigned int i = 0; i < binList.size() - 1; i++) {
		ss << "From " << (i * (range / bins)) << " - "
				<< ((i + 1) * (range / bins));
		ss << ";";
		ss << binList.at(i) << std::endl;
	}

	ss << "From " << range << ";";
	ss << binList.at(binList.size() - 1) << std::endl;

	return ss.str();
}
