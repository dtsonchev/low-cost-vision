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
	/**
	 * Creates a new ReportField with a specified name
	 * @param name the name for the field
	 */
	ReportField(std::string name) :
			fieldName(name) {
	}

	/**
	 * All children must implement toString
	 * @return the string representation of the field
	 */
	virtual std::string toString() = 0;

	/**
	 * Gets the name of the field
	 * @return the current name of the field
	 */
	std::string getFieldName() const {
		return fieldName;
	}
	/**
	 * Sets the name of the field
	 * @param fieldName the new name for the field
	 */
	void setFieldName(std::string fieldName) {
		this->fieldName = fieldName;
	}

	/**
	 * All children must implement setColumnNames,
	 * as only the child knows the number of columns
	 * @param first the name of the first columns
	 */
	virtual void setColumnNames(const char*  first, ...) = 0;
	/**
	 * Returns the names of the columns
	 * @return the names of the columns
	 */
	std::vector<const char*> getColumnNames(){return columnNames;}

	/**
	 * Destructor
	 */
	virtual ~ReportField(){}

protected:
	std::string fieldName;
	std::vector<const char*> columnNames;
};

}
