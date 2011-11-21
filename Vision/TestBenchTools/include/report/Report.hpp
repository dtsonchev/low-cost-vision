#pragma once
#include <vector>
#include <report/ReportField.hpp>

namespace report {

class Report{
public:
	void saveHTML(const std::string& path);

	void addField(const ReportField& field);

	std::vector<std::vector<std::string> > splitText(const std::string& text);

private:
	std::vector<ReportField*> fields;
};

}
