#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <report/Report.hpp>
#include <report/Html.hpp>

#ifdef __CDT_PARSER__
#define foreach(a, b) for(a : b)
#else
#define foreach(a, b) BOOST_FOREACH(a, b)
#endif

void report::Report::addField(ReportField* field){
	fields.push_back(field);
}

std::vector<std::vector<std::string> > report::Report::splitText(const std::string& text, std::string anyOf){
	using namespace std;
	using namespace boost::algorithm;

	stringstream ss(text);
	string s;
	vector<vector<string> > lines;

	getline(ss, s);
	while(!ss.fail()){
		std::vector<std::string> words;
		split(words, s, is_any_of(anyOf));
		if(words[words.size() - 1] == ""){
			words.pop_back();
		}
		lines.push_back(words);
		getline(ss, s);
	}

	return lines;
}

bool report::Report::saveHTML(const std::string& pt){
	using namespace std;
	using namespace boost::filesystem;

	path p(pt);
	if(!exists(p.parent_path())){
		create_directory(p.parent_path());
	}

	ofstream out(pt.c_str());
	if(!out.is_open()){
		return false;
	}
	out << HTML_START;

	out << HTML_H1_START << title << HTML_H1_END;
	out << HTML_P_START << description << HTML_P_END;

	foreach(ReportField* field, fields){
		out << HTML_TABLE_START;
		out << HTML_TABLE_CAPTION_START << field->getFieldName() << HTML_TABLE_CAPTION_END;

		if(!field->getColumnNames().empty()){
			out << HTML_TABLE_ROW_START;
			foreach(string header, field->getColumnNames()){
				out << HTML_TABLE_HEADER_START << header << HTML_TABLE_HEADER_END;
			}
			out << HTML_TABLE_ROW_END;
		}

		vector<vector<string> > lines = splitText(field->toString(), ";");
		foreach(vector<string> line, lines){
			out << HTML_TABLE_ROW_START;
			foreach(string word, line){
				out << HTML_TABLE_CELL_START << word << HTML_TABLE_CELL_END;
			}
			out << HTML_TABLE_ROW_END;
		}
		out << HTML_TABLE_END;
	}

	out << HTML_END;
	out.flush();
	out.close();

	if(out.fail()){
		return false;
	}
	return true;
}

