#pragma once
#include <string>
#include <report/ReportList.hpp>
#include <report/ReportField.hpp>
#include <imageMetaData/Types.hpp>

namespace report {
/**
 * This class is a ReportField that represents the results of a certain category
 */
class CategoryOverview : public ReportList{
public:
	/**
	 * Creates an overview from a CategoryResults object.
	 * @param cr the CategoryResults from the overview is created
	 */
	CategoryOverview(const imageMetaData::CategoryResults& cr);
};

}
