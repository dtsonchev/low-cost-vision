#pragma once
#include <vector>
#include <report/ReportField.hpp>

namespace report {
/**
 * This class contains a full report
 */
class Report{
public:
	/**
	 * Save the report to a file with html formatting
	 * @param path the full path of the file to save to
	 */
	void saveHTML(const std::string& path);
	/**
	 * Add a ReportField to the report
	 * @param field the field to add
	 */
	void addField(ReportField* field);
	
private:
	std::vector<ReportField*> fields;
	std::vector<std::vector<std::string> > splitText(const std::string& text, std::string anyOf);
	};

}
