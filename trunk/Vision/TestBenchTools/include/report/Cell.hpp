#pragma once
#include <string>

namespace report {

////////////////////////////////////////////////////////
enum Type { INT,DOUBLE,STRING};
//////////////////////////////////////////////////////

class Cell {
public:
	Cell(Type t) : type (t) {}
	virtual std::string toString() {return "";}
	virtual ~Cell(){}
private:
	Type type;
};

}
