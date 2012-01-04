/* File : testlib.i */
%module testlib
%include "std_string.i"
using namespace std;
%{
	class aClass {
	public:
		aClass(int i);
		void printiD();
	private:
		int iD;
	};
	void outprintiD(aClass ff);
	void doSomething(std::string s);
	std::string returnSomething(std::string s);
%}

class aClass {
public:
	aClass(int i) ;
	void printiD();
private:
	int iD;
};
void outprintiD(aClass ff);
void doSomething(std::string s);
std::string returnSomething(std::string s);
