#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>

void childCallback(const std_msgs::String::ConstPtr& msg)
{
	const char * fullMessage = msg->data.c_str()+1;
	const char * message=0;
	int id;
	std::stringstream ss;
	while((*fullMessage>='0' && *fullMessage<='9' &&
			*fullMessage != '\0' ) || *fullMessage == ']' ) {
		if(*fullMessage == ']') {
			message = fullMessage+1;
			break;
		}
		ss << *fullMessage;
		fullMessage++;
	}
	ss >> id;
	ROS_INFO("Line1: a child with MASTId %i sent: %s",id,message);
}

int rosrun(const char * package,const char * executable,const char * arg1 ) {
	int forkValue = fork();
	switch(forkValue) {
		case 0: execlp("rosrun","rosrun",package,executable,arg1,NULL);
		break;
		default: wait();
		break;
	}
return forkValue;
}

int main(int argc, char **argv){
	const int mastId = 10;
	ros::init(argc, argv, "Line1");
	ros::NodeHandle n;
	ROS_INFO("Line1 created...");
	ros::Publisher line1Pub = n.advertise<std_msgs::String>("TopicLine1", 1000);
	usleep(350*1000);
	std_msgs::String msg;
	std::stringstream ss;
	ss << "created";
	msg.data = ss.str();
	line1Pub.publish(msg);
	rosrun("MastTree", "Cell2_1", "TopicLine1");
	ros::Subscriber c2_1Sub = n.subscribe("TopicCell2_1", 1000, childCallback);
	rosrun("MastTree", "Cell2_2", "TopicLine1");
	ros::Subscriber c2_2Sub = n.subscribe("TopicCell2_2", 1000, childCallback);
	//TODO: rosrun Cell2_2
	ros::spin();
	return 0;
}
