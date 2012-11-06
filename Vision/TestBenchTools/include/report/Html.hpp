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
