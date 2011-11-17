#pragma once

#include <string>
#include <sstream>
#include "Cell.hpp"


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

class DoubleCell : public Cell {
public:
	DoubleCell(double val) : Cell(DOUBLE), value(val) {}
	double getValue() { return value;}
	virtual ~DoubleCell(){};
	std::string toString() { std::stringstream ss; ss << value; return ss.str(); }
private:
	double value;
};

class StringCell : public Cell {
public:
	StringCell(std::string val) : Cell(STRING), value(val) {}
	virtual ~StringCell(){}
	std::string getValue() { return value;}
	std::string toString() { return value; }
private:
	std::string value;
};

//TODO:Telsell

