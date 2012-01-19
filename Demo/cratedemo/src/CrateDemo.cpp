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
#include <vision/getAllCrates.h>
#include <deltarobotnode/gripper.h>
#include <cratedemo/GridCrate4x4MiniBall.hpp>
#include <cratedemo/MotionWrapper.hpp>

namespace cratedemo
{
	CrateDemo::CrateDemo(
		ros::NodeHandle& hNode,
		const std::string& deltaGrip,
		const std::string& deltaStop,
		const std::string& deltaMotion,
		const std::string& deltaError,
		const std::string& crateRefresh,
		const std::string& visionEvents,
		const std::string& visionError,
		CrateContentMap& crateContentMap) :
			gripperClient( hNode.serviceClient<deltarobotnode::gripper>(deltaGrip) ),
			motionClient( hNode.serviceClient<deltarobotnode::motion>(deltaMotion) ),
			stopClient(hNode.serviceClient<deltarobotnode::stop>(deltaStop)),
			crateRefreshClient(hNode.serviceClient<vision::getAllCrates>(crateRefresh)),
			deltaErrorSub(hNode.subscribe(deltaError, 1000, &CrateDemo::deltaErrorCb, this)),
			crateEventSub(hNode.subscribe(visionEvents, 1000, &CrateDemo::crateEventCb, this)),
			visionErrorSub(hNode.subscribe(visionError, 1000, &CrateDemo::visionErrorCb, this)),
			crateContentMap(crateContentMap) {	}

void CrateDemo::deltaErrorCb(const deltarobotnode::error::ConstPtr& msg)
{
	ROS_ERROR("Delta node error[%i]:\t%s",msg->errorType,msg->errorMsg.c_str());
	onDeltaError(msg->errorType, msg->errorMsg);
}

void CrateDemo::visionErrorCb(const vision::error::ConstPtr& msg)
{
	ROS_ERROR("Vision node error[%i]:\t%s", msg->errorType, msg->errorMsg.c_str());
	onVisionError(msg->errorType, msg->errorMsg);
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
	case IN: handleNewCrate(msg->crate); break;
	case MOVED: handleCrateMoved(msg->crate); break;
	case MOVING: handleCrateMoving(msg->crate); break;
	case OUT: handleCrateRemoved(msg->crate); break;
	default: assert(0 && "unknown crate event"); break;
	}
}

void CrateDemo::handleNewCrate(const vision::CrateMsg& msg)
{
	bool moving = false;
	CrateContentMap::iterator it = crateContentMap.find(msg.name);
	if(it != crateContentMap.end()){
		ROS_ERROR("NewCrate: Crate content unknown, crate ignored: %s", msg.name.c_str());
		return;
	}

	Crate* crate = new GridCrate4x4MiniBall(msg.name, it->second,datatypes::point2f(msg.x,msg.y),msg.angle, moving);
	crates.insert(std::pair<std::string, Crate*>(crate->getName(), crate));

	onNewCrate(*crate);
}

void CrateDemo::handleCrateRemoved(const vision::CrateMsg& msg) {
	std::map<std::string, Crate*>::iterator result = crates.find(msg.name);

	if(result != crates.end()){
		ROS_WARN("CrateRemoved: Crate didn't exist, remove action ignored: %s", msg.name.c_str());
		return;
	}

	onCrateRemoved(*(result->second));
	delete result->second;
	crates.erase(result);
}

void CrateDemo::handleCrateMoved(const vision::CrateMsg& msg)
{
	CrateMap::iterator res = crates.find(msg.name);
	if(res != crates.end()){
		vision::getAllCrates::Response allCrates;
		vision::getAllCrates::Request req;
		crateRefreshClient.call(req, allCrates);

		for(int i = 0; i < allCrates.crates.size(); i++){
			if(allCrates.crates.at(i).name == msg.name){
				handleNewCrate(allCrates.crates.at(i));
				ROS_WARN("CrateMoved: Crate didn't exist, crate added: %s", msg.name.c_str());
				break;
			}
		}
	}

	Crate* c = res->second;
	c->position = datatypes::point2f(msg.x,msg.y);
	c->angle = msg.angle;
	c->moving = false;
	onCrateMove(*c);
}

void CrateDemo::handleCrateMoving(const vision::CrateMsg& msg)
{
	CrateMap::iterator res = crates.find(msg.name);
	assert(res != crates.end() && "crate does not exist!");
	Crate* c = res->second;
	c->moving = true;
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

//struct MotionCtor
//{
//	deltarobotnode::motion motions;
//
//	void addMotion(const datatypes::point3f& p, float speed)
//	{
//		motions.request.x.push_back(p.x );
//		motions.request.y.push_back(p.y );
//		motions.request.z.push_back(p.z );
//		motions.request.speed.push_back(speed);
//	}
//
//	bool move(ros::ServiceClient& client)
//	{
//		client.call(motions);
//		return motions.response.succeeded;
//	}
//};

/*void CrateDemo::drawCrateCorners(Crate& crate) {
	ROS_INFO("*-*-*-*-*-*-*-*-*-*-*-*-*-*-drawCrateCorners-*-*-*-*-*-*-*-*-*-*-*-*-*-*");
	const int speed = 150;
	//MotionCtor ctor;
	MotionWrapper motionw;

	datatypes::point2f pointLT = datatypes::point2f(-(crate.size.width/2),-(crate.size.depth/2));
	pointLT = pointLT.rotate(crate.angle);
	pointLT += crate.position;
	motionw.addMotion(datatypes::point3lf(pointLT.x,pointLT.y,SAFE_HEIGHT),speed);
	motionw.addMotion(datatypes::point3lf(pointLT.x,pointLT.y,TABLE_HEIGHT + 15),speed);
	motionw.addMotion(datatypes::point3lf(pointLT.x,pointLT.y,SAFE_HEIGHT),speed);

	datatypes::point2f pointRT = datatypes::point2f((crate.size.width/2),-(crate.size.depth/2));
	pointRT = pointRT.rotate(crate.angle);
	pointRT += crate.position;
	motionw.addMotion(datatypes::point3lf(pointRT.x,pointRT.y,SAFE_HEIGHT),speed);
	motionw.addMotion(datatypes::point3lf(pointRT.x,pointRT.y,TABLE_HEIGHT + 15),speed);
	motionw.addMotion(datatypes::point3lf(pointRT.x,pointRT.y,SAFE_HEIGHT),speed);

	datatypes::point2f pointLB = datatypes::point2f(-(crate.size.width/2),(crate.size.depth/2));
	pointLB = pointLB.rotate(crate.angle);
	pointLB += crate.position;
	motionw.addMotion(datatypes::point3lf(pointLB.x,pointLB.y,SAFE_HEIGHT),speed);
	motionw.addMotion(datatypes::point3lf(pointLB.x,pointLB.y,TABLE_HEIGHT + 15),speed);
	motionw.addMotion(datatypes::point3lf(pointLB.x,pointLB.y,SAFE_HEIGHT),speed);

	datatypes::point2f pointRB = datatypes::point2f((crate.size.width/2),(crate.size.depth/2));
	pointRB = pointRB.rotate(crate.angle);
	pointRB += crate.position;
	motionw.addMotion(datatypes::point3lf(pointRB.x,pointRB.y,SAFE_HEIGHT),speed);
	motionw.addMotion(datatypes::point3lf(pointRB.x,pointRB.y,TABLE_HEIGHT + 15),speed);
	motionw.addMotion(datatypes::point3lf(pointRB.x,pointRB.y,SAFE_HEIGHT),speed);

	motionw.print();

	if(!motionw.callService(motionClient))
	{
		ROS_WARN("Can't touch this!");
	}

	ROS_INFO("*-*-*-*-*-*-*-*-*-*-*-*-*-*-drawCrateCorners-*-*-*-*-*-*-*-*-*-*-*-*-*-*");
}*/
void CrateDemo::moveObject(Crate& crateFrom, size_t indexFrom ,Crate& crateTo, size_t indexTo ){
	//get positions
	datatypes::point3f posFrom = crateFrom.getContentGripLocation(indexFrom);
	datatypes::point3f posTo = crateTo.getContainerLocation(indexTo) + crateFrom.get(indexFrom)->getGripPoint();
	const double speed = 350;

	//update crate database
	CrateContent* c = crateFrom.get(indexFrom);
	crateTo.put(indexTo, c);
	crateFrom.remove(indexFrom);
	
	//move to source, down
	MotionWrapper motion1;
	motion1.addMotion(datatypes::point3f(posFrom.x, posFrom.y, SAFE_HEIGHT), speed);
	motion1.addMotion(posFrom, speed);
	motion1.callService(motionClient);

	//enable gripper
//	deltarobotnode::gripper grip;
//	grip.request.enabled = true;
//	gripperClient.call(grip);

	//move up, dest, down
	MotionWrapper move2;
	move2.addMotion(datatypes::point3f(posFrom.x, posFrom.y, SAFE_HEIGHT), speed);
	move2.addMotion(datatypes::point3f(posTo.x, posTo.y, SAFE_HEIGHT), speed);
	move2.addMotion(posTo, speed);
	move2.callService(motionClient);

	//Drop object
//	grip.request.enabled = false;
//	gripperClient.call(grip);

	//loose ball //TODO: this is temporary, delete it
	MotionWrapper move4;
	move4.addMotion(datatypes::point3f(posTo.x + 3, posTo.y + 3, posTo.z + 3), speed);
	move4.callService(motionClient);

	//move up
	MotionWrapper move3;
	move3.addMotion(datatypes::point3f(posTo.x, posTo.y, SAFE_HEIGHT), speed);
	move3.callService(motionClient);
}
}
