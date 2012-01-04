#ifndef MASTENTITY_H
#define MASTENTITY_H

#include <iostream>
#include <string>
#include <vector>
#include"MAST.h"

using namespace std;

class MastEntity {
public:
	MastEntity(int id, string nme);
	MastEntity(int id, string nme, Status stat );
	MastEntity(int id, string nme, Status stat, vector<ChildStatus> cSL );
	void changeStatus(Status stat);
	void addChild(ChildStatus cldSt);
	void updateChildrenStatusList(ChildStatus cldSt);
	virtual ~MastEntity();
private:
	int mastId;
	char* name;
	Status status;
	vector<ChildStatus> childrenStatusList;
};

#endif
