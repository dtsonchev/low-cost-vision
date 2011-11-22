#pragma once
#include <string>
#include <report/ReportList.hpp>
#include <report/ReportField.hpp>
#include <imageMetaData/Types.hpp>

namespace report {
/**
 * Creates an overview of a CategoryResults.
 */
class CategoryOverview : public ReportField{
public:
	/**
	 * Creates an overview of a CategoryResults.
	 * @param cr the CategoryResults on which the overview is created
	 */
	CategoryOverview(const imageMetaData::CategoryResults& cr);
	/**
	 * Returns the std::string representation of the CategoryResults
	 * @return std::string
	 */
	std::string toString();
private:
	ReportList reportlist;
};

}
