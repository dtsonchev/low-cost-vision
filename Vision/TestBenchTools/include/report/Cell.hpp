#pragma once
#include <string>

namespace report {

////////////////////////////////////////////////////////
enum Type { INT,DOUBLE,STRING};
//////////////////////////////////////////////////////

/**
 * Base Class of IntCell, DoubleCell and StringCell
 */
class Cell {
public:
	Cell(Type t) : type (t) {}
	virtual std::string toString() {return "";}
	virtual ~Cell(){}
private:
	Type type;
};

}
