#include <vector>
#include <CategoryOverview.hpp>
#include <ReportField.hpp>
#include <ReportList.hpp>
#include <boost/foreach.hpp>

#include <sstream>
#ifdef __CDT_PARSER__
    #define foreach(a, b) for(a : b)
#else
    #define foreach(a, b) BOOST_FOREACH(a, b)
#endif

CatergoryOverview::CatergoryOverview(const ImageMetaData::CategoryResults& cr) : ReportField(cr.first) {
	//typedef std::pair<std::string, std::map<std::string, std::pair<int, int> > > CategoryResults;
	name = cr.first;

	std::vector<Type> columns;
	columns.push_back(STRING);
	columns.push_back(INT);
	columns.push_back(STRING);
	reportlist = ReportList(columns);
	std::stringstream percent;
	foreach(ImageMetaData::SubCategoryResults scr, cr.second){
		percent.str("");
		percent << ((scr.second.first *100 )/ scr.second.second) << "%";
		reportlist.appendRow(scr.first.c_str(), scr.second.first,percent.str().c_str());
	}

}
std::string CatergoryOverview::toString() {
	std::stringstream ss;
	ss << name  << ":" << std::endl;
	ss << reportlist.toString();
	return ss.str();
}
