#ifndef SUBSCRIBERS_H
#define SUBSCRIBERS_H

#include <ros/ros.h>

struct Subscribers{
	ros::Subscriber joySubscriber_;
    ros::Subscriber positionSubscriber_;
    ros::Subscriber obstacleSubscriber_;
    ros::Subscriber modeSubscriber_;
    ros::Subscriber targetsFoundSubscriber_;
    ros::Subscriber targetsCollectedSubscriber_;
    ros::Subscriber odometrySubscriber_;
};


#endif