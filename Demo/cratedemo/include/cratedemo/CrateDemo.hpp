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
#include <deltarobotnode/motion.h>
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
