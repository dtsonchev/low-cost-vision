#pragma once
#include <report/SubCells.hpp>

namespace report {

	/**
	 * Creates a new IntegerCell with value
	 * @param val the value to put in the cell
	 * @return the new cell
	 */
	static IntegerCell* newCell(int val) { return new IntegerCell(val); }
	/**
	 * Creates a new DoubleCell with value
	 * @param val the value to put in the cell
	 * @return the new cell
	 */
	static DoubleCell* newCell(double val) { return new DoubleCell(val); }
	/**
	 * Creates a new StringCell with value
	 * @param val the value to put in the cell
	 * @return the new cell
	 */
	static StringCell* newCell(const char * val) { return new StringCell(val); }
	/**
	 * Creates a new StringCell with value
	 * @param val the value to put in the cell
	 * @return the new cell
	 */
	static StringCell* newCell(std::string& val) { return new StringCell(val.c_str()); }
}
