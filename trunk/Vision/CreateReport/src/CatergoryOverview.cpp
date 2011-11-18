#include <vector>
#include <CategoryOverview.hpp>
#include <ReportField.hpp>
#include <ReportList.hpp>
#include <boost/foreach.hpp>

#ifdef __CDT_PARSER__
    #define foreach(a, b) for(a : b)
#else
    #define foreach(a, b) BOOST_FOREACH(a, b)
#endif

CatergoryOverview::CatergoryOverview(const ImageMetaData::CategoryResults& cr) : ReportField(cr.first) {
	//typedef std::pair<std::string, std::map<std::string, std::pair<int, int> > > CategoryResults;


	std::vector<Type> columns;
	columns.push_back(STRING);
	columns.push_back(INT);
	columns.push_back(INT);
	reportlist = ReportList(columns);
	int percent;
	foreach(ImageMetaData::SubCategoryResults scr, cr.second){
		percent = (scr.second.first *100 / scr.second.second);
		reportlist.appendRow(scr.first.c_str(),scr.second,percent);
	}

}
std::string CatergoryOverview::toString() {
	return reportlist.toString();
}
