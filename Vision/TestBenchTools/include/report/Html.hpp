//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        TestBenchTools
// File:           Html.hpp
// Description:    Definitions for some regular html tags.
// Author:         Franc Pape
// Notes:          ...
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

#define HTML_START "\
<!doctype html public \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n\
<html>\n\
	<head>\n\
		<title>Results</title>\n\
		<style type=\"text/css\">\n\
			body{font-size: 14px;}\n\
			h1{font-size:2em; margin:0.5em 0 0;}\n\
			p{margin:0 0 1em;}\n\
			table,th,td {border:1px solid black; border-spacing:0; border-collapse:collapse;}\n\
			table {margin-bottom:1em;}\n\
			th,td {padding:0.1em 0.2em; text-align:center;}\n\
			th {font-weight:bold; font-size:1em;}\n\
			caption {text-align:left; font-weight:bold; font-size:1.2em;}\n\
		</style>\n\
	</head>\n\
	\n\
	<body>\n\
"
#define HTML_END "\
	</body>\n\
</html>\n\
"
#define HTML_H1_START "<h1>"
#define HTML_H1_END "</h1>\n"
#define HTML_P_START "<p>"
#define HTML_P_END "</p>\n"
#define HTML_TABLE_START "<table>\n"
#define HTML_TABLE_END "</table>\n"
#define HTML_TABLE_ROW_START "<tr>\n"
#define HTML_TABLE_ROW_END "</tr>\n"
#define HTML_TABLE_CELL_START "<td>"
#define HTML_TABLE_CELL_END "</td>\n"
#define HTML_TABLE_CAPTION_START "<caption>"
#define HTML_TABLE_CAPTION_END "</caption>\n"
#define HTML_TABLE_HEADER_START "<th>"
#define HTML_TABLE_HEADER_END "</th>\n"
#define HTML_ULIST_START "<ul>"
#define HTML_ULIST_END "</ul>"
#define HTML_LIST_ITEM_START "<li>"
#define HTML_LIST_ITEM_END "</li>"
