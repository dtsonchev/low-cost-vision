#include "Cell.hpp"
#include "subCells.hpp"
#include "ReportList.hpp"

#include <iostream>
using namespace std;

int main(int argc, char** argv) {

	using std::vector;
	vector<Type> col;
	col.push_back(STRING);
	col.push_back(INT);
	col.push_back(DOUBLE);
	ReportList reportList(col);
	const char * one = "one";
	int two = 2;
	double three = 3.3;
	reportList.appendRow(one,&two,&three,NULL);
	reportList.appendRow(one,&two,&three,NULL);
	reportList.appendRow(one,&two,&three,NULL);
	reportList.appendRow(one,&two,&three,NULL);
	reportList.appendRow(one,&two,&three,NULL);
	reportList.appendRow(one,&two,&three,NULL);
	reportList.appendRow(one,&two,&three,NULL);
	reportList.appendRow(one,&two,&three,NULL);
	reportList.enableAverageRow(true);
	reportList.enableSumRow(true);
	cout << reportList.toString();
	return 0;
}
