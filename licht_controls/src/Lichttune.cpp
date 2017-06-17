#include "ros/ros.h"
#include <std_msgs/Int32.h>
int main(int argc, char **argv)
{
	ros::init(argc, argv, "tune");
	ros::NodeHandle n;
	ros::Rate loop_rate(1);
	int index;
	n.getParam("/tune_index", index);
	ros::Publisher tune_index_pub = n.advertise<std_msgs::Int32>("/tune_index",1);
	std_msgs::Int32 msg;
	msg.data = index;
	ROS_INFO("%d",msg.data);
	int cnt=0;
	while (ros::ok())
	{
		
		cnt++;
		if(cnt>2)
			break;
		tune_index_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
//	ros::spin();
	return 0;
}