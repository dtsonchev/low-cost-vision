//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        TestBenchTools
// File:           CategoryOverview.hpp
// Description:    This class is a ReportField that represents the results of a certain category.
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
#include <string>
#include <report/ReportList.hpp>
#include <report/ReportField.hpp>
#include <imageMetaData/Types.hpp>

namespace report {
/**
 * This class is a ReportField that represents the results of a certain category
 */
class CategoryOverview : public ReportList{
public:
	/**
	 * Creates an overview from a CategoryResults object.
	 * @param cr the CategoryResults from the overview is created
	 */
	CategoryOverview(const imageMetaData::CategoryResults& cr);
};

}
