#pragma once
#include <vector>
#include <report/ReportField.hpp>

namespace report {
/**
 * Class which contains a full report
 */
class Report{
public:
	/***
	 * save the report to a html-file
	 */
	void saveHTML(const std::string& path);
	/***
	 * Add a reportfield to the report
	 */
	void addField(ReportField* field);
	
private:
	/**
	 * List of reportfields in the report
	 */
	std::vector<ReportField*> fields;
	//TODO: doxygen
	std::vector<std::vector<std::string> > splitText(const std::string& text, std::string anyOf);
	};

}
