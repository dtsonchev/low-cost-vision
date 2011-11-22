#pragma once
#include <string>
#include <sstream>
#include <report/Cell.hpp>

namespace report {
/**
 * Integer representation of the base class Cell
 */
class IntegerCell : public Cell {
public:
	IntegerCell(int val) : Cell(INT), value(val) {}
	int getValue() { return value;}
	std::string toString() {
		std::stringstream ss;
		ss << value;
		return ss.str();
	}
private:
	int value;
};

/**
 * Double representation of the base class Cell
 */
class DoubleCell : public Cell {
public:
	DoubleCell(double val) : Cell(DOUBLE), value(val) {}
	double getValue() { return value;}
	virtual ~DoubleCell(){};
	std::string toString() { std::stringstream ss; ss << value; return ss.str(); }
private:
	double value;
};

/**
 * String representation of the base class Cell
 */
class StringCell : public Cell {
public:
	StringCell(std::string val) : Cell(STRING), value(val) {}
	virtual ~StringCell(){}
	std::string getValue() { return value;}
	std::string toString() { return value; }
private:
	std::string value;
};

}
