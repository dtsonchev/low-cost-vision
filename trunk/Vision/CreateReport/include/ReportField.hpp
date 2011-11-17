#pragma once

#include <string>

class ReportField {

	public:
		ReportField() : fieldName("noName") {};
		ReportField(std::string name) : fieldName(name) {};
		virtual std::string toString() =0;

		virtual ~ReportField(){}

	private:
		std::string fieldName;
};
