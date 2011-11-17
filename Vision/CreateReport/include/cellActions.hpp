#pragma once

#include "subCells.hpp"

namespace report {
	using std::string;
	static IntegerCell * newCell(int val) { return new IntegerCell(val); }
	static DoubleCell  * newCell(double val) { return new DoubleCell(val); }
	static StringCell  * newCell(const char * val) { return new StringCell(val); }
}
