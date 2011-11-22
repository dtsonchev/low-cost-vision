#pragma once
#include <string>
#include <vector>

namespace report {

/**
 * Base class of the objects available to reports
 */
class ReportField {
public:
	/**
	 * Creates a new reportfield with name 'noName'
	 */
	ReportField() :
			fieldName("noName") {
	}
	/***
	 * Creates a new reportfield with std::string name as name
	 */
	ReportField(std::string name) :
			fieldName(name) {
	}
	/**
	 * pure virtual function forces the child classes to implement this method
	 */
	virtual std::string toString() = 0;
	
	virtual ~ReportField(){}

	

	/**
	 * getter for the name of the field
	 */
	std::string getFieldName() const {
		return fieldName;
	}
	/**
	 * setter for the name of the field
	 */
	void setFieldName(std::string fieldName) {
		this->fieldName = fieldName;
	}

	virtual void setColumnNames(const char*  first, ...) = 0;
	std::vector<const char*> getColumnNames(){return columnNames;}

protected:
	std::string fieldName;
	std::vector<const char*> columnNames;
};

}
