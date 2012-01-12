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

#include <cratedemo/Crate.hpp>

#include <ros/ros.h>

//delta node messages & services
#include <deltarobotnode/error.h>
#include <deltarobotnode/gripper.h>
#include <deltarobotnode/motion.h>
#include <deltarobotnode/stop.h>

//WOOO 133333337!!!!!!!111111 one
#include <visDum/CrateEventMsg.h>

namespace cratedemo
{
typedef std::map<std::string, Crate*> CrateMap;

/**
 * Framework for a demo with crates.
 */
class CrateDemo
{
private:
	static const float SAFE_HEIGHT = -140;
	static const float TABLE_HEIGHT = -200;
	// deltarobot services
	ros::ServiceClient gripperClient;
	ros::ServiceClient motionClient;
	ros::ServiceClient stopClient;
	ros::Subscriber deltaErrorSub;

	// vision services
	ros::Subscriber crateEventSub;

	CrateContentMap& crateContentMap;
	void newCrateCb(const visDum::CrateMsg& msg);
	void crateRemovedCb(const visDum::CrateMsg& msg);
	void crateMovedCb(const visDum::CrateMsg& msg);

protected:
	CrateDemo(
		ros::NodeHandle& hNode,
		const std::string& deltaGrip,
		const std::string& deltaStop,
		const std::string& deltaMotion,
		const std::string& deltaError,
		const std::string& visionEvents,
		CrateContentMap& crateContentMap);

	CrateMap crates;


public:
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
/**
 * Updates
 */
	//void GeneralCb(/*message*/);
	void update();
	void moveObject(Crate& crateFrom, size_t indexFrom ,Crate& crateTo, size_t indexTo);
	void crateEventCb(const visDum::CrateEventMsg::ConstPtr& msg);
	void deltaErrorCb(const deltarobotnode::error::ConstPtr& msg);

///////testing/////////////////////////
void  CrateDance(Crate& crate);
};
}
