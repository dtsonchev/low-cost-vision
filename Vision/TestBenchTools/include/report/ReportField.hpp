#pragma once
#include <string>

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
	virtual std::string toString() =0;
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

	virtual ~ReportField() {
	}

protected:
	std::string fieldName;
};

}
