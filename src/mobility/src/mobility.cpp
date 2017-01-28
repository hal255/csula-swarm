
//ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>

//ROS messages
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

//Custom messages
#include <shared_messages/TagsImage.h>

// To handle shutdown signals so the node quits properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

#include <string>

#include "MobilityHelper.h"
#include "Subscribers.h"
#include "Publishers.h"
#include "Timers.h"
#include "Stoke.h"
#include "Rover.h"



//Mobility Logic Functions
void setVelocity(double linearVel, double angularVel);

// state machine states
#define STATE_MACHINE_TRANSFORM	0
#define STATE_MACHINE_ROTATE	1
#define STATE_MACHINE_TRANSLATE	2
int stateMachineState = STATE_MACHINE_TRANSFORM;

char host[128];



void setPublisherName(std::string&, int, char**);
void sigintEventHandler(int signal);

int main(int argc, char **argv) {


	std::string publishedName;
	Rover* r2d2 = new Rover();
	geometry_msgs::Pose2D goalLocation;
    random_numbers::RandomNumberGenerator* rng = new random_numbers::RandomNumberGenerator();

    setPublisherName(publishedName, argc, argv);
    goalLocation.x = .1;
    goalLocation.y = .1;
    r2d2->target_detected_.data = -1; //initialize target detected
	r2d2->setGoalPosition(goalLocation);
	r2d2->setHostName(publishedName);

    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (publishedName + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;

    MobilityHelper mh(r2d2, &mNH);

    signal(SIGINT,sigintEventHandler); // Register the SIGINT event handler so the node can shutdown properly

    mh.subs->joySubscriber_ = mNH.subscribe((publishedName + "/joystick"), 10, &MobilityHelper::joyCmdHandler, &mh);
    mh.subs->modeSubscriber_ = mNH.subscribe((publishedName + "/mode"), 1, &MobilityHelper::modeHandler, &mh);
    mh.subs->targetsFoundSubscriber_ = mNH.subscribe((publishedName + "/targets"), 10, &MobilityHelper::targetHandler, &mh);
    mh.subs->obstacleSubscriber_ = mNH.subscribe((publishedName + "/obstacle"), 10, &MobilityHelper::obstacleHandler, &mh);
    mh.subs->odometrySubscriber_ = mNH.subscribe((publishedName + "/odom/ekf"), 10, &MobilityHelper::odometryHandler, &mh);
    mh.subs->targetsCollectedSubscriber_ = mNH.subscribe(("targetsCollected"), 10, &MobilityHelper::targetsCollectedHandler, &mh);

    mh.pubs->status_publisher = mNH.advertise<std_msgs::String>((publishedName + "/status"), 1, true);
    mh.pubs->velocityPublish = mNH.advertise<geometry_msgs::Twist>((publishedName + "/velocity"), 10);
    mh.pubs->stateMachinePublish = mNH.advertise<std_msgs::String>((publishedName + "/state_machine"), 1, true);
    mh.pubs->targetsCollectedPublish = mNH.advertise<std_msgs::Int16>(("targetsCollected"), 1, true);
    mh.pubs->targetPickUpPublish = mNH.advertise<sensor_msgs::Image>((publishedName + "/targetPickUpImage"), 1, true);
    mh.pubs->targetDropOffPublish = mNH.advertise<sensor_msgs::Image>((publishedName + "/targetDropOffImage"), 1, true);

    mh.timers->publish_status_timer = mNH.createTimer(ros::Duration(5), &MobilityHelper::publishStatusTimerEventHandler, &mh);
    mh.timers->killSwitchTimer = mNH.createTimer(ros::Duration(10), &MobilityHelper::killSwitchTimerEventHandler, &mh);
    mh.timers->stateMachineTimer = mNH.createTimer(ros::Duration(0.1), &MobilityHelper::search, &mh);
    
    ros::spin();

    return EXIT_SUCCESS;
}

void sigintEventHandler(int signal)
{
	ros::shutdown();
}

void setPublisherName(std::string& publishedName, int argc, char** argv){
	gethostname(host, sizeof (host));
	std::string hostname(host);


	if (argc >= 2) {
        publishedName = argv[1];
        std::cout << "Welcome to the world of tomorrow " << publishedName << "!  Mobility module started." << std::endl;
    } else {
        publishedName = hostname;
        std::cout << "No Name Selected. Default is: " << publishedName << std::endl;

    }
}

