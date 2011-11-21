#pragma once
#include <string>
#include <report/ReportList.hpp>
#include <report/ReportField.hpp>
#include <imageMetaData/Types.hpp>

namespace report {

class CategoryOverview : public ReportField{
public:
	CategoryOverview(const imageMetaData::CategoryResults& cr);
	std::string toString();
private:
	ReportList reportlist;
};

}
