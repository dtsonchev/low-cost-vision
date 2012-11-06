//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        MastTreeTestCase
// File:           GenericMASTEntity.cpp
// Description:    The goal of this project is to implement the MAst architecture and to test its feasibility.
// Author:         Wouter Langerak & Franc Pape
// Notes:          This was only a small test, it has to be rewritten if implemented in a real system.
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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <vector>
#include <stdarg.h>
//Max depth of the tree.
#define MAXDEPTH 4
using namespace std;

ros::Publisher* pubPnt;
ros::Time begin;
//parameters
int mastId ,noOfChildren,currentLevel,childMsgCount;
	string parentTopicName;
	stringstream nameStream;
	stringstream topicNameStream;
	stringstream nodeName;

void childCallback(const std_msgs::String::ConstPtr& msg)
{
	stringstream pubStream;
	if(msg->data[0] == '^' ){
		if(++childMsgCount == noOfChildren && currentLevel > 0) {
			pubStream << "^" << mastId << "^" << "message from child: " << msg->data.c_str();
			std_msgs::String mg;
			mg.data = pubStream.str();
			pubPnt->publish(mg);
		}
		else if(currentLevel==0) {
			pubStream << "^" << mastId << "^i am on the top, child said: " << msg->data.c_str();
			ROS_INFO("%s",pubStream.str().c_str());
			if(childMsgCount == noOfChildren ){
				ros::Duration relapsed = ros::Time::now() - begin;
				ROS_INFO("relapsed milisec: %i",(relapsed.nsec /1000 /1000));
			}
		}
	}
}

void parentCallback(const std_msgs::String::ConstPtr& msg)
{
	if(msg->data[0] == '+' && msg->data[2]) {
		begin = ros::Time::now();
		childMsgCount =0;
	}
	stringstream pubStream;
	if(msg->data[0] == '+' ){
		childMsgCount =0;
		if(currentLevel<MAXDEPTH) {
			pubStream << "+" << mastId << "+" << "message from parent: " << msg->data.c_str();
			std_msgs::String mg;
			mg.data = pubStream.str();
			pubPnt->publish(mg);
		}
		else {
			pubStream << "^" << mastId << "^leaf reached";
			std_msgs::String mg;
			mg.data = pubStream.str();
			pubPnt->publish(mg);
			//ROS_INFO("%s",pubStream.str().c_str());
		}
	}
}
//Last parameter must be NULL
void rosrun(const char * package,const char * executable, ...) {

	va_list args;
	va_start( args, executable );
	const char * mg =0;
	stringstream cmd;
	cmd << "rosrun " << package << " " << executable;
	for(mg = va_arg(args,const char*);mg!=0;mg = va_arg(args,const char*)) {
		cmd << " " << mg;
	}
	va_end( args );
	int sys;
	switch(fork()){
		case 0: sys = system(cmd.str().c_str());
		break;
		case -1: ROS_ERROR("fork() for creating new node failed");
		break;
		default: //Do Nothing
		break;
	}


}

int main(int argc, char **argv){
	ros::init(argc, argv, "tempNodeName");
	ros::NodeHandle n;
	//start import parameters
	nodeName << ros::this_node::getName() << "/";
	stringstream paramStream;
	paramStream << nodeName.str()  << "mastId";
	n.getParam(paramStream.str().c_str(),mastId);
	paramStream.str("");
	paramStream << nodeName.str() << "parentTopicName";
	n.getParam(paramStream.str().c_str(),parentTopicName);
	paramStream.str("");
	paramStream << nodeName.str() << "noOfChildren";
	n.getParam(paramStream.str().c_str(),noOfChildren);
	paramStream.str("");
	paramStream << nodeName.str() << "currentLevel";
	n.getParam(paramStream.str().c_str(),currentLevel);
	//end import parameters

	nameStream << "Entity" << mastId;
	topicNameStream << "TopicOf" << nameStream.str();
	vector<ros::Subscriber> childSubscriberList(noOfChildren);

	/*//ROS_INFO("%s created:\n "
			"mastId: %i \n"
			"noOfChildren: %i \n"
			"currentLevel: %i \n"
			"parentTopicName: %s",nameStream.str().c_str(),mastId,noOfChildren,currentLevel,parentTopicName.c_str());

*/
	ros::Publisher pub = n.advertise<std_msgs::String>(topicNameStream.str(), 1000);
	pubPnt = &pub;
	usleep(350*1000);
	std_msgs::String msg;
	std::stringstream ss;
	ss << "["<< mastId << "]created";
	msg.data = ss.str();
	pub.publish(msg);

	ros::Subscriber parentTopic;
	if(currentLevel>0){
		parentTopic = n.subscribe(parentTopicName, 1000, parentCallback);
		//ROS_INFO("[%i]subscribed to parent!->%s",mastId,parentTopicName.c_str());

	}
	if(currentLevel< MAXDEPTH) {
		stringstream childIdStream, childName, childNameParam , childTopicName,noOfChildrenStream, childLevel,parentTopicStream;
		noOfChildrenStream << "_noOfChildren:=" << noOfChildren;
		childLevel << "_currentLevel:=" << (currentLevel+1);
		parentTopicStream << "_parentTopicName:=" << topicNameStream.str();
		for(int i=0;i<noOfChildren;i++) {
			////ROS_INFO("%d * %d", mastId + 1, pow(10, log10(noOfChildren) + 1));
			int childId = (mastId == 0 ? 1 : mastId) * pow(10, floor(log10(noOfChildren)) + 1) + i;
			//int childId = mastId * 100 + i;
			childIdStream.str("");
			childIdStream << "_mastId:=" << childId;
			childName.str("");
			childName << "Entity" << childId;
			childTopicName.str("");
			childTopicName << "TopicOf" << childName.str();
			childNameParam.str("");
			childNameParam << " __name:=" << childName.str();
			//Rosrun(package, executable, nodeName, mastID , partentTopic , noOfChildren , currentLevel)
			/*//ROS_INFO("create child: MastTreeTestCase GenericMASTEntity %s %s %s %s %s",
								childNameParam.str().c_str(),
								childIdStream.str().c_str(),
								parentTopicStream.str().c_str(),
								noOfChildrenStream.str().c_str(),
								childLevel.str().c_str());
*/
			//Rosrun(package, executable, nodeName, mastID , partentTopic , noOfChildren , currentLevel)
			rosrun("MastTreeTestCase", "GenericMASTEntity",
					childNameParam.str().c_str(),
					childIdStream.str().c_str(),
					parentTopicStream.str().c_str(),
					noOfChildrenStream.str().c_str(),
					childLevel.str().c_str(),NULL
			);
			childSubscriberList.at(i) = n.subscribe(childTopicName.str().c_str() , 1000, childCallback);
			////ROS_INFO("[%i] subscribed to: %s",mastId, childSubscriberList.at(i-1).getTopic().c_str());
		}
	}else {
		ROS_INFO("This branch is done...");
	}

	ros::spin();
	return 0;
}
