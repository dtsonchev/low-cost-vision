#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sstream>
using namespace std;
void sendOutcomeToBench(string varType, string varName, string varValue) {
	stringstream ss;
	ss << "<type>" << varType << "<type><name>" << varName << "<name><value>" << varValue << "<value>";
	printf("%s",ss.str().c_str());
}

int main(int argc, char** argv) {
	sendOutcomeToBench("int","intVar","42");
	return 0;
}
