//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        CreateReport
// File:           ReportList.hpp
// Description:    This class is the most basic ReportField containing a matrix of values.
// Author:         Wouter Langerak
// Notes:          
//
// License: newBSD 
//  
// Copyright © 2012, HU University of Applied Sciences Utrecht. 
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
// - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//******************************************************************************

#pragma once

#include <string>
#include <vector>
#include <ReportField.hpp>
#include <Cell.hpp>

class ReportList : public ReportField {

public:
	ReportList();
	ReportList(std::vector<Type> col);
	void appendRow(const int  cell1, ...);
	void appendRow(const double cell1, ...);
	void appendRow(const char * cell1, ...);
	void enableSumRow(bool sum);
	void enableAverageRow(bool avg);
	void enablePercentRow(bool perc, unsigned int partRow, unsigned int fullRow);
	std::string toString();
private:
	bool hasSumRow;
	bool hasAverageRow;
	bool hasPercentRow;
	unsigned int numberOfColumns;
	unsigned int percPartRow;
	unsigned int percFullRow;
	std::vector<Type> columns;
	std::vector< std::vector<Cell*> > data;
	void appendCell(const int column, const int value);
	void appendCell(const int column,const double value);
	void appendCell(const int column,const char * value);
	void appendCell(const int column, std::string value);
	void appendSumRowToSs(std::stringstream& ss);
	void appendAverageRowToSs(std::stringstream& ss);
	void appendPercentRowToSs(std::stringstream& ss);

};

