#ifndef TIMERS_H
#define TIMERS_H

#include <ros/ros.h>

struct Timers{
	// Pubs
	ros::Timer publish_status_timer;
	ros::Timer killSwitchTimer;
	ros::Timer stateMachineTimer;
};

#endif