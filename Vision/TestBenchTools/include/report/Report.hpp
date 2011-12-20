//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        TestBenchTools
// File:           Report.hpp
// Description:    This class contains a full report.
// Author:         Wouter Langerak
// Notes:          
//
// License:        GNU GPL v3
//
// This file is part of TestBenchTools.
//
// TestBenchTools is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// TestBenchTools is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with TestBenchTools.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#pragma once
#include <vector>
#include <report/ReportField.hpp>

namespace report {
/**
 * This class contains a full report
 */
class Report{
public:
	/**
	 * Constructor that sets the title of the report
	 * @param title the title for the report
	 */
	Report(std::string title) : title(title){	}
	/**
	 * Save the report to a file with html formatting
	 * @param path the full path of the file to save to
	 * @return true if saved successfully, false otherwise
	 */
	bool saveHTML(const std::string& path);
	/**
	 * Add a ReportField to the report
	 * @param field the field to add
	 */
	void addField(ReportField* field);
	/**
	 * Sets the description of this report
	 * @param desc the description
	 */
	void setDescription(std::string desc){description=desc;}
	
private:
	std::vector<ReportField*> fields;
	std::string title;
	std::string description;

	std::vector<std::vector<std::string> > splitText(const std::string& text, std::string anyOf);
	};

}
