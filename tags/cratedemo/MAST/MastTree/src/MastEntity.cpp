
#include "MastEntity.h"
#include <string>
#include <vector>

using namespace std;

MastEntity::MastEntity(int id, string nme) {
	mastId = id;
	name = nme;
	status = NULL;
	childrenStatusList = NULL;
}
MastEntity::MastEntity(int id, string nme, Status stat) {
	mastId = id;
	name = nme;
	status = stat;
	childrenStatusList = NULL;
}

MastEntity::MastEntity(int id, string nme, Status stat, vector<ChildStatus> cSL ) {
	mastId = id;
	name = nme;
	status = stat;
	childrenStatusList = cSL;
}
void MastEntity::changeStatus(Status stat) {
	status = stat;
}

void MastEntity::addChild(ChildStatus cldSt) {
	childrenStatusList.at(childrenStatusList.size()) = cldSt;
}

void MastEntity::updateChildrenStatusList(ChildStatus cldSt) {
	//TODO update status of parameterchild
	/*TODO if(cldSt.Status == Start || cldSt.Status == Stop) {
		if( all kids changed) {
			notify parent
		}
	}
}

MastEntity::~MastEntity() {
	// TODO Auto-generated destructor stub
}

