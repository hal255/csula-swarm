#ifndef PUBLISHERS_H
#define PUBLISHERS_H

#include <ros/ros.h>

struct Publishers{
	// Pubs
	ros::Publisher targetsFoundPublish;
	ros::Publisher targetsCollectedPublish;
	ros::Publisher velocityPublish;
	ros::Publisher positionPublisher;
	ros::Publisher status_publisher;
	ros::Publisher stateMachinePublish;
	ros::Publisher targetPickUpPublish;
	ros::Publisher targetDropOffPublish;
	ros::Publisher joyPublish;
};



#endif