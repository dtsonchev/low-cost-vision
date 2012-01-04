#include <iostream>
#include <string.h>
using namespace std;

class aClass {
	public:
		aClass(int i) {
			iD = i;
		}
		void printiD() {
			cout << iD << endl;
		}
	private:
		int iD;
};

void doSomething(string s) {
	cout << "testlib: I did something with:" << s << endl;
}

void outprintiD(aClass ff) {
	ff.printiD();
}
string returnSomething(string s) {
	return s;
}
//Don't know why, but without the next line it doesn't work. :(
aClass z = aClass(1);

