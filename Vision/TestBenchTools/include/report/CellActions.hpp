#pragma once
#include <report/SubCells.hpp>

namespace report {
	/**
	 * returns a new IntegerCell with value
	 * @param val
	 * @return
	 */
	static IntegerCell * newCell(int val) { return new IntegerCell(val); }
	/**
	 * returns a new DoubleCell with value
	 * @param val
	 * @return
	 */
	static DoubleCell  * newCell(double val) { return new DoubleCell(val); }
	/**
	 * returns a new StringCell with value
	 * @param val
	 * @return
	 */
	static StringCell  * newCell(const char * val) { return new StringCell(val); }

}
