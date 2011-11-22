#include <stdarg.h>
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

report::CategoryOverview::CategoryOverview(const imageMetaData::CategoryResults& cr) :
		ReportList(cr.first.c_str(), 3, STRING, INT, STRING) {
	std::stringstream percent;
	foreach(imageMetaData::SubCategoryResults scr, cr.second){
		percent.str("");
		percent << ((scr.second.first *100 )/ scr.second.second) << "%";
		appendRow(scr.first.c_str(), scr.second.first,percent.str().c_str());
	}

}
