#pragma once
#include <Crate.h>
#include <map>
#include <vector>
#include <string>

class CrateEvent{
public:
	enum crate_event_type
	{
		type_in = 1,
		type_out = 2,
		type_moving = 3,
		type_moved = 4
	};

	CrateEvent(crate_event_type type = type_moving, std::string name = "", float x = 0, float y = 0, float angle = 0)
		: type(type), name(name), x(x), y(y), angle(angle){}

	~CrateEvent() {};

	std::string toString(){
		std::stringstream ss;
		std::string typeString;
		switch(type){
		case type_in : typeString = "In";break;
		case type_out : typeString = "Out";break;
		case type_moving : typeString = "Moving";break;
		case type_moved : typeString = "Moved";break;
		}
		ss << "CrateEvent: \n\ttype: "<< typeString <<"\n\tName: "<< name <<"\n\tX: " << x << "\n\tY: "<< y <<"\n\tAngle: "<< angle;
		return ss.str();
	}

	int type;
	std::string name;
	float x, y, angle;
};

class exCrate : public Crate{
public:
	exCrate (const Crate& crate, int framesLeft = 0):
	Crate(crate), oldSituation(false), newSituation(true), exists(true), stable(false), framesLeft(framesLeft){}

	~exCrate() {};

	bool oldSituation, newSituation, exists, stable;
	int framesLeft;
};


class crateTracker{
public:
	crateTracker(int stableFrames, double movementThresshold);
	~crateTracker(){};

	std::vector<CrateEvent> update(std::vector<Crate> crates);

	int stableFrames;
	double movementThresshold;
	double rotationThresshold;
private:
	bool hasChanged(const Crate& newCrate,const Crate& oldCrate);
	std::map<std::string,exCrate> knownCrates;

};
