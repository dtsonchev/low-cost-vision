#pragma once
#include <string>

namespace report {

class ReportField {
public:
	ReportField() : fieldName("noName") {}
	ReportField(std::string name) : fieldName(name) {}
	virtual std::string toString() =0;
	std::string getFieldName() const { return fieldName; }
	void setFieldName(std::string fieldName){ this->fieldName = fieldName; }

	virtual ~ReportField(){}

protected:
	std::string fieldName;
};

}
