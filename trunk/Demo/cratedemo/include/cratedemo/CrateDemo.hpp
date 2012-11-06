//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        cratedemo
// File:           CrateDemo.hpp
// Description:    Framework for a demo with crates.
// Author:         Lukas Vermond
// Notes:
//
// License: newBSD 
//  
// Copyright © 2012, HU University of Applied Sciences Utrecht. 
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
// - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//******************************************************************************

#pragma once

#include <string>
#include <vector>
#include <map>
#include <queue>

#include <cratedemo/MoveAction.hpp>
#include <cratedemo/Crate.hpp>
#include <cratedemo/Environment.hpp>
#include <ros/ros.h>
#include <boost/thread.hpp>

//delta node messages & services
#include <deltarobotnode/error.h>
#include <deltarobotnode/gripper.h>
#include <deltarobotnode/motionSrv.h>
#include <deltarobotnode/stop.h>

//WOOO 133333337!!!!!!!111111 one
#include <vision/CrateEventMsg.h>
#include <vision/error.h>
#include <vision/getCrate.h>

namespace cratedemo
{
typedef std::map<std::string, Crate*> CrateMap;

/**
 * Framework for a demo with crates.
 */
class CrateDemo
{
private:
	// deltarobot services/topics
	ros::ServiceClient gripperClient;
	ros::ServiceClient motionClient;
	ros::ServiceClient checkClient;
	ros::ServiceClient stopClient;
	ros::Subscriber deltaErrorSub;

	// vision services/topics
	ros::ServiceClient crateRefreshClient;
	ros::ServiceClient getCrateClient;
	ros::Subscriber crateEventSub;
	ros::Subscriber visionErrorSub;

	std::queue<MoveAction> actionQueue;
	CrateContentMap& crateContentMap;

	volatile bool threadRunning;
	boost::mutex actionQueueMutex;
	boost::recursive_mutex crateMapMutex;

	volatile bool idle;
	boost::mutex idleMutex;
	boost::condition_variable idleCondition;

	boost::mutex waitMutex;
	boost::condition_variable waitCondition;

	boost::thread* actionThread;
protected:
	CrateMap crates;
private:
	//crate event handling methods
	void handleNewCrate(const vision::CrateMsg& msg);
	void handleCrateRemoved(const vision::CrateMsg& msg);
	void handleCrateMoved(const vision::CrateMsg& msg);
	void handleCrateMoving(const vision::CrateMsg& msg);

	/**
	 * @note needs crateMapMutex to be locked
	 */
	Crate* waitForCrate(const std::string& name);
	datatypes::point3f getCrateContentGripLocation(const Crate& crate, size_t index);

	static void staticActionThreadFunc(CrateDemo* obj);
	void actionThreadFunc(void);
	//void drawCrateCorners(Crate& crate); //for debugging

protected:
	CrateDemo(
		ros::NodeHandle& hNode,
		const std::string& deltaGrip,
		const std::string& deltaStop,
		const std::string& deltaMotion,
		const std::string& checkMotion,
		const std::string& deltaError,
		const std::string& crateRefresh,
		const std::string& getCrate,
		const std::string& visionEvents,
		const std::string& visionError,
		CrateContentMap& crateContentMap);

public:
	void getAllCrates(void);
	/**
	 *Deconstructor
	 */
	virtual ~CrateDemo();

	/**
	 * Method which is called after the (vision) system detects a new crate.
	 * @param crate
	 */
	virtual void onNewCrate(Crate& crate) = 0;
	/**
	 * Method which is called after the (vision) system detects that a crate is moved.
	 * @param crate
	 */
	virtual void onCrateMove(Crate& crate) = 0;
	virtual void onCrateRemoved(Crate& crate) = 0;
	virtual void onDeltaError(int errCode, const std::string& errStr) = 0;
	virtual void onVisionError(int errCode, const std::string& errStr) = 0;

	void moveObject(Crate& crateFrom, size_t indexFrom ,Crate& crateTo, size_t indexTo);
	void crateEventCb(const vision::CrateEventMsg::ConstPtr& msg);
	void deltaErrorCb(const deltarobotnode::error::ConstPtr& msg);
	void visionErrorCb(const vision::error::ConstPtr& msg);
};
}
