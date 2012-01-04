#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>

void parentCallback(const std_msgs::String::ConstPtr& msg)
{

}
int main(int argc, char **argv){
	const int mastId = 31;
	ros::init(argc, argv, "Module3_1");
	ros::NodeHandle n;
	ROS_INFO("Module3_1 created...");
	ros::Publisher pub = n.advertise<std_msgs::String>("TopicModule3_1", 1000);
	usleep(350*1000);
	std_msgs::String msg;
	std::stringstream ss;
	ss << "[" << mastId << "]created";
	msg.data = ss.str();
	pub.publish(msg);
	ros::Subscriber parentTopic = n.subscribe(argv[1], 1000, parentCallback);
	//TODO: rosrun Module3.1
	//TODO: rosrun Module3.2
	ros::spin();
	return 0;
}
