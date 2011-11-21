#include <vector>
#include <sstream>
#include <boost/foreach.hpp>
#include <report/CategoryOverview.hpp>
#include <report/ReportField.hpp>
#include <report/ReportList.hpp>

#ifdef __CDT_PARSER__
    #define foreach(a, b) for(a : b)
#else
    #define foreach(a, b) BOOST_FOREACH(a, b)
#endif

report::CategoryOverview::CategoryOverview(const imageMetaData::CategoryResults& cr) : ReportField(cr.first) {
	//typedef std::pair<std::string, std::map<std::string, std::pair<int, int> > > CategoryResults;

	std::vector<Type> columns;
	columns.push_back(STRING);
	columns.push_back(INT);
	columns.push_back(STRING);
	reportlist = ReportList(columns);
	std::stringstream percent;
	foreach(imageMetaData::SubCategoryResults scr, cr.second){
		percent.str("");
		percent << ((scr.second.first *100 )/ scr.second.second) << "%";
		reportlist.appendRow(scr.first.c_str(), scr.second.first,percent.str().c_str());
	}

}
std::string report::CategoryOverview::toString() {
	return reportlist.toString();
}
