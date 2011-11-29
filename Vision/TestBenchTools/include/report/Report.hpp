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
	 * Constructor that sets the title of the report
	 * @param title the title for the report
	 */
	Report(std::string title) : title(title){	}
	/**
	 * Save the report to a file with html formatting
	 * @param path the full path of the file to save to
	 * @return true if saved successfully, false otherwise
	 */
	bool saveHTML(const std::string& path);
	/**
	 * Add a ReportField to the report
	 * @param field the field to add
	 */
	void addField(ReportField* field);
	/**
	 * Sets the description of this report
	 * @param desc the description
	 */
	void setDescription(std::string desc){description=desc;}
	
private:
	std::vector<ReportField*> fields;
	std::string title;
	std::string description;

	std::vector<std::vector<std::string> > splitText(const std::string& text, std::string anyOf);
	};

}
