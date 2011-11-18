#pragma once
#include <string>
#include <ReportList.hpp>
#include <ReportField.hpp>
#include <ImageMD/Types.hpp>
class CatergoryOverview : public ReportField{
public:
	CatergoryOverview(const ImageMetaData::CategoryResults& cr);
	std::string toString();
private:
	ReportList reportlist;
};
