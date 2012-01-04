#ifndef MAST_H
#define MAST_H

using namespace std;

enum Status {
	Normal,
	Standby,
	Safe,
	Set_Up,
	Start,
	Stop,
	Shutdown
};

struct ChildStatus{
	int mastId;
	Status status;
};
#endif
