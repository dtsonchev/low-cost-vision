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
}

CrateDemo::~CrateDemo()
{
}


void CrateDemo::crateEventCb(const visDum::CrateEventMsg::ConstPtr& msg)
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
	case IN: newCrateCb(msg->crate); break;
	case MOVED: crateMovedCb(msg->crate); break;
	case MOVING: //TODO implements
	break;
	case OUT: crateRemovedCb(msg->crate); break;
	default: break;
	}
}

void CrateDemo::newCrateCb(const visDum::CrateMsg& msg)
{
	CrateContentMap::iterator it = crateContentMap.find(msg.name);
	Crate* crate = new GridCrate4x4MiniBall(msg.name, it->second,datatypes::point2f(msg.x,msg.y),msg.angle);
	crates.insert(std::pair<std::string, Crate*>(crate->getName(), crate));

	onNewCrate(*crate);
}

void CrateDemo::crateRemovedCb(const visDum::CrateMsg& msg) {
	std::map<std::string, Crate*>::iterator result = crates.find(msg.name);
	onCrateRemoved(*(result->second));
	delete result->second;
	crates.erase(result);
}

void CrateDemo::crateMovedCb(const visDum::CrateMsg& msg)
{
	std::map<std::string, Crate*>::iterator result = crates.find(msg.name);
	Crate* c= result->second;
	c->position = datatypes::point2f(msg.x,msg.y);
	c->angle = msg.angle;
	onCrateMove(*c);
}

void CrateDemo::update()
{
	ros::spinOnce();
}

void CrateDemo::moveObject(Crate& crateFrom, size_t indexFrom ,Crate& crateTo, size_t indexTo ){
	datatypes::point3f posFrom = crateFrom.getCrateContentGripLocation(indexFrom);
	datatypes::point3f posTo = crateTo.getContainerLocation(indexTo) + crateFrom.get(indexFrom)->getGripPoint();
	const int speed = 350;
	//move to source
	deltarobotnode::motion move1;
	move1.request.x.push_back(posFrom.x);
	move1.request.y.push_back(posFrom.y);
	move1.request.z.push_back(SAFE_HEIGHT);
	move1.request.speed.push_back(speed);
	//move down
	move1.request.x.push_back(posFrom.x);
	move1.request.y.push_back(posFrom.y);
	move1.request.z.push_back(posFrom.z);
	move1.request.speed.push_back(speed);
	motionClient.call(move1);
/*
	//enable gripper
	deltarobotnode::gripper grip;
	grip.request.enabled = true;
	gripperClient.call(grip);
*/
	//move up
	deltarobotnode::motion move2;
	move2.request.x.push_back(posFrom.x);
	move2.request.y.push_back(posFrom.y);
	move2.request.z.push_back(SAFE_HEIGHT);
	move2.request.speed.push_back(speed);
	//move dest
	move2.request.x.push_back(posTo.x);
	move2.request.y.push_back(posTo.y);
	move2.request.z.push_back(SAFE_HEIGHT);
	move2.request.speed.push_back(speed);
	//move down
	move2.request.x.push_back(posTo.x);
	move2.request.y.push_back(posTo.y);
	move2.request.z.push_back(posTo.z);
	move2.request.speed.push_back(speed);
	motionClient.call(move2);
/*
	//Drop object
	grip.request.enabled = false;
	gripperClient.call(grip);
*/
	//move up
	deltarobotnode::motion move3;
	move3.request.x.push_back(posTo.x);
	move3.request.y.push_back(posTo.y);
	move3.request.z.push_back(SAFE_HEIGHT);
	move3.request.speed.push_back(speed);
	motionClient.call(move3);

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
