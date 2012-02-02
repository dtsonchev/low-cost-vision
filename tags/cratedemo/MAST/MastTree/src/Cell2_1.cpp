#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>

ros::Publisher* pubPnt;

void childCallback(const std_msgs::String::ConstPtr& msg)
{
	const char * fullMessage = msg->data.c_str()+1;
	const char * message= 0;
	int childId;
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
	ss >> childId;
	ROS_INFO("Cell2_1: a child with MASTId %i sent: %s",childId,message);
	std_msgs::String sendMsg;
	std::stringstream sendStream;
	sendStream << "[21]" << childId << message;
	sendMsg.data = sendStream.str();
	pubPnt->publish(sendMsg);
}

void parentCallback(const std_msgs::String::ConstPtr& msg)
{

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
	const int mastId = 21;
	ros::init(argc, argv, "Cell2_1");
	ros::NodeHandle n;
	ROS_INFO("Cell2_1 created...");
	ros::Publisher pub = n.advertise<std_msgs::String>("TopicCell2_1", 1000);
	pubPnt = &pub;
	usleep(350*1000);
	std_msgs::String msg;
	std::stringstream ss;
	ss << "["<< mastId << "]created";
	msg.data = ss.str();
	pub.publish(msg);
	ros::Subscriber parentTopic = n.subscribe(argv[1], 1000, parentCallback);
	rosrun("MastTree", "Module3_1", "TopicCell2_1");
	ros::Subscriber m3_1Sub = n.subscribe("TopicModule3_1", 1000, childCallback);
	//TODO: rosrun Module3.1
	//TODO: rosrun Module3.2
	ros::spin();
	return 0;
}
