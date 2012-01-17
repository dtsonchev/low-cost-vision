//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        cratedemo
// File:           CrateDemo.cpp
// Description:    Framework for a demo with crates.
// Author:         Lukas Vermond
// Notes:
//
// License:        GNU GPL v3
//
// This file is part of cratedemo.
//
// cratedemo is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// cratedemo is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with cratedemo.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#include <cratedemo/CrateDemo.hpp>

#include <cassert>
#include <iostream>

#include <ros/ros.h>
#include <deltarobotnode/gripper.h>
#include <cratedemo/GridCrate4x4MiniBall.hpp>
namespace cratedemo
{
	CrateDemo::CrateDemo(
		ros::NodeHandle& hNode,
		const std::string& deltaGrip,
		const std::string& deltaStop,
		const std::string& deltaMotion,
		const std::string& deltaError,
		const std::string& visionEvents,
		CrateContentMap& crateContentMap) :
			gripperClient( hNode.serviceClient<deltarobotnode::gripper>(deltaGrip) ),
			motionClient( hNode.serviceClient<deltarobotnode::motion>(deltaMotion) ),
			stopClient(hNode.serviceClient<deltarobotnode::stop>(deltaStop)),
			deltaErrorSub(hNode.subscribe(deltaError, 1000, &CrateDemo::deltaErrorCb, this)),
			crateEventSub(hNode.subscribe(visionEvents, 1000, &CrateDemo::crateEventCb, this)),
			crateContentMap(crateContentMap) {	}

void CrateDemo::deltaErrorCb(const deltarobotnode::error::ConstPtr& msg)
{
	ROS_ERROR("Delta robot error[%i]:\t%s",msg->errorType,msg->errorMsg.c_str());
	onDeltaError(msg->errorType, msg->errorMsg);
}

CrateDemo::~CrateDemo()
{
}

void CrateDemo::crateEventCb(const vision::CrateEventMsg::ConstPtr& msg)
{
	enum
	{
		IN=1,
		OUT,
		MOVING,
		MOVED
	};

	switch(msg->event)
	{
	case IN: handleNewCrateCb(msg->crate); break;
	case MOVED: handleCrateMoved(msg->crate); break;
	case MOVING: handleCrateMoving(msg->crate); break;
	case OUT: handleCcrateRemoved(msg->crate); break;
	default: assert(0 && "unknown crate event"); break;
	}
}

void CrateDemo::handleNewCrate(const vision::CrateMsg& msg)
{
	CrateContentMap::iterator it = crateContentMap.find(msg.name);
	assert(it != crateContentMap.end() && "crate content not found!"); //TODO refresh cratedb and try again
	Crate* crate = new GridCrate4x4MiniBall(msg.name, it->second,datatypes::point2f(msg.x,msg.y),msg.angle);
	crates.insert(std::pair<std::string, Crate*>(crate->getName(), crate));

	onNewCrate(*crate);
}

void CrateDemo::handleCrateRemoved(const vision::CrateMsg& msg) {
	std::map<std::string, Crate*>::iterator result = crates.find(msg.name);
	assert(result != crates.end() && "crate does not exist!"); //TODO refresh cratedb and try again
	onCrateRemoved(*(result->second));
	delete result->second;
	crates.erase(result);
}

void CrateDemo::handleCrateMoved(const vision::CrateMsg& msg)
{
	CrateMap::iterator res = crates.find(msg.name);
	assert(result != crates.end() && "crate does not exist!"); //TODO refresh crate db and try again
	Crate* c = result->second;
	c->position = datatypes::point2f(msg.x,msg.y);
	c->angle = msg.angle;
	onCrateMove(*c);
}

void CrateDemo::handleCrateMoving(const vision::CrateMsg& msg)
{
	//TODO impl
}

//DEBUG
static void printMove(const deltarobotnode::motion& move)
{
	std::cout << "printMove invoked" << std::endl;
	if(move.request.x.size() == move.request.y.size() && move.request.x.size() == move.request.z.size() && move.request.x.size() == move.request.speed.size())
	{
		for(size_t i = 0; i < move.request.x.size(); i++)
		{
			std::cout << "i=" << i << " x=" << move.request.x.at(i) << " y=" << move.request.y.at(i) << " z=" << move.request.z.at(i) << " speed=" << move.request.speed.at(i) << std::endl;
		}
	}
	else
	{
		std::cout << "vectors not of same size!" << std::endl;
	}
	std::cout << "printMove returned" << std::endl;
}

struct MotionCtor
{
	deltarobotnode::motion motions;

	void addMotion(const datatypes::point3f& p, float speed)
	{
		motions.request.x.push_back(p.x );
		motions.request.y.push_back(p.y );
		motions.request.z.push_back(p.z );
		motions.request.speed.push_back(speed);
	}

	bool move(ros::ServiceClient& client)
	{
		client.call(motions);
		return motions.response.succeeded;
	}
};

void CrateDemo::drawCrateCorners(Crate& crate) {
	ROS_INFO("*-*-*-*-*-*-*-*-*-*-*-*-*-*-drawCrateCorners-*-*-*-*-*-*-*-*-*-*-*-*-*-*");
	const int speed = 150;
	MotionCtor ctor;
	datatypes::point2f pointLT = datatypes::point2f(-(crate.size.width/2),-(crate.size.depth/2));
	pointLT = pointLT.rotate(crate.angle);
	pointLT += crate.position;
	ctor.addMotion(datatypes::point3f(pointLT.x,pointLT.y,SAFE_HEIGHT),speed);
	ctor.addMotion(datatypes::point3f(pointLT.x,pointLT.y,TABLE_HEIGHT + 15),speed);
	ctor.addMotion(datatypes::point3f(pointLT.x,pointLT.y,SAFE_HEIGHT),speed);

	datatypes::point2f pointRT = datatypes::point2f((crate.size.width/2),-(crate.size.depth/2));
	pointRT = pointRT.rotate(crate.angle);
	pointRT += crate.position;
	ctor.addMotion(datatypes::point3f(pointRT.x,pointRT.y,SAFE_HEIGHT),speed);
	ctor.addMotion(datatypes::point3f(pointRT.x,pointRT.y,TABLE_HEIGHT + 15),speed);
	ctor.addMotion(datatypes::point3f(pointRT.x,pointRT.y,SAFE_HEIGHT),speed);

	datatypes::point2f pointLB = datatypes::point2f(-(crate.size.width/2),(crate.size.depth/2));
	pointLB = pointLB.rotate(crate.angle);
	pointLB += crate.position;
	ctor.addMotion(datatypes::point3f(pointLB.x,pointLB.y,SAFE_HEIGHT),speed);
	ctor.addMotion(datatypes::point3f(pointLB.x,pointLB.y,TABLE_HEIGHT + 15),speed);
	ctor.addMotion(datatypes::point3f(pointLB.x,pointLB.y,SAFE_HEIGHT),speed);

	datatypes::point2f pointRB = datatypes::point2f((crate.size.width/2),(crate.size.depth/2));
	pointRB = pointRB.rotate(crate.angle);
	pointRB += crate.position;
	ctor.addMotion(datatypes::point3f(pointRB.x,pointRB.y,SAFE_HEIGHT),speed);
	ctor.addMotion(datatypes::point3f(pointRB.x,pointRB.y,TABLE_HEIGHT + 15),speed);
	ctor.addMotion(datatypes::point3f(pointRB.x,pointRB.y,SAFE_HEIGHT),speed);

	printMove(ctor.motions);

	if(!ctor.move(motionClient))
	{
		ROS_WARN("Can't touch this!");
	}

	//motionClient.call(ctor.motions);

	ROS_INFO("*-*-*-*-*-*-*-*-*-*-*-*-*-*-drawCrateCorners-*-*-*-*-*-*-*-*-*-*-*-*-*-*");
}
void CrateDemo::moveObject(Crate& crateFrom, size_t indexFrom ,Crate& crateTo, size_t indexTo ){
	datatypes::point3f posFrom = crateFrom.getCrateContentGripLocation(indexFrom);
	datatypes::point3f posTo = crateTo.getContainerLocation(indexTo) + crateFrom.get(indexFrom)->getGripPoint();
	const int speed = 350;
	//move to source
//	deltarobotnode::motion move1;
//	move1.request.x.push_back(posFrom.x);
//	move1.request.y.push_back(posFrom.y);
//	move1.request.z.push_back(SAFE_HEIGHT);
//	move1.request.speed.push_back(speed);

	MotionCtor ctor;
	ctor.addMotion(datatypes::point3f(posFrom.x, posFrom.y, SAFE_HEIGHT), speed);
	ctor.addMotion(posFrom, speed);

	//move down
//	move1.request.x.push_back(posFrom.x);
//	move1.request.y.push_back(posFrom.y);
//	move1.request.z.push_back(posFrom.z);
//	move1.request.speed.push_back(speed);

	//printMove(move1);

	motionClient.call(ctor.motions);//move1);

	//enable gripper
	/*deltarobotnode::gripper grip;
	grip.request.enabled = true;
	gripperClient.call(grip);*/

	//move up
//	deltarobotnode::motion move2;
//	move2.request.x.push_back(posFrom.x);
//	move2.request.y.push_back(posFrom.y);
//	move2.request.z.push_back(SAFE_HEIGHT);
//	move2.request.speed.push_back(speed);
//	//move dest
//	move2.request.x.push_back(posTo.x);
//	move2.request.y.push_back(posTo.y);
//	move2.request.z.push_back(SAFE_HEIGHT);
//	move2.request.speed.push_back(speed);
//	//move down
//	move2.request.x.push_back(posTo.x);
//	move2.request.y.push_back(posTo.y);
//	move2.request.z.push_back(posTo.z);
//	move2.request.speed.push_back(speed);

	MotionCtor ctor1;
	ctor1.addMotion(datatypes::point3f(posFrom.x, posFrom.y, SAFE_HEIGHT), speed);
	ctor1.addMotion(datatypes::point3f(posTo.x, posTo.y, SAFE_HEIGHT), speed);
	ctor1.addMotion(posTo, speed);

	motionClient.call(ctor1.motions);//move2);

	//Drop object
	/*grip.request.enabled = false;
	gripperClient.call(grip);*/

	//move up
//	deltarobotnode::motion move3;
//	move3.request.x.push_back(posTo.x);
//	move3.request.y.push_back(posTo.y);
//	move3.request.z.push_back(SAFE_HEIGHT);
//	move3.request.speed.push_back(speed);

	MotionCtor ctor2;
	ctor2.addMotion(datatypes::point3f(posTo.x, posTo.y, SAFE_HEIGHT), speed);
	motionClient.call(ctor2.motions);//move3);

	CrateContent* c = crateFrom.get(indexFrom);
	crateTo.put(indexTo, c);
	crateFrom.remove(indexFrom);
}

void CrateDemo::CrateDance(Crate& crate) {
	datatypes::point3f posFrom = crate.getCrateContentGripLocation(0);
	datatypes::point3f posTo = crate.getContainerLocation(1) + crate.get(0)->getGripPoint();
	const int speed = 50;
//move 0
	deltarobotnode::motion move1;
	move1.request.x.push_back(posFrom.x);
	move1.request.y.push_back(posFrom.y);
	move1.request.z.push_back(posFrom.z);
	move1.request.speed.push_back(speed);
	motionClient.call(move1);

	deltarobotnode::motion move2;
	move2.request.x.push_back(posTo.x);
	move2.request.y.push_back(posTo.y);
	move2.request.z.push_back(posTo.z);
	move2.request.speed.push_back(speed);
	motionClient.call(move2);
}
}
