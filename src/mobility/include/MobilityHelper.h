#ifndef MOBILITYHELPER_H
#define MOBILITYHELPER_H

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
#include <shared_messages/TagsImage.h>

// To handle shutdown signals so the node quits properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

struct Publishers;
struct Subscribers;
struct Timers;
class Rover;
class Uniform;
class TagStore;
class Initialization;
class Stoke;
class Timers;
enum StateMachineState {TRANSFORM, ROTATE, TRANSLATE};

class MobilityHelper
{
public:
    MobilityHelper(Rover* rover, ros::NodeHandle* nh);
    virtual void search(const ros::TimerEvent&);
    void setRover(Rover* rover);
    //Callback handlers
    void joyCmdHandler(const geometry_msgs::Twist::ConstPtr& message);
    void modeHandler(const std_msgs::UInt8::ConstPtr& message);
    //Virtual functions. You Don't need to implement these. Only implement
    //these if you need them to do something else
    virtual void targetHandler(const shared_messages::TagsImage::ConstPtr& message);
    virtual void obstacleHandler(const std_msgs::UInt8::ConstPtr& message);
    virtual void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
    //void returnTagHandler(const std_msgs::String);
    virtual void publishStatusTimerEventHandler(const ros::TimerEvent& event);
    virtual void targetsCollectedHandler(const std_msgs::Int16::ConstPtr& message);
    virtual void killSwitchTimerEventHandler(const ros::TimerEvent& event);
    // virtual void positionHandler(const geometry_msgs::Pose2D::ConstPtr& message);
    // virtual void positionHandler(const std_msgs::Odometry::ConstPtr& message);

    Publishers*	 pubs;
    Subscribers* subs;
    Timers* 	 timers;
    Uniform*	 uniform_;

protected:
    Rover*				 rover_;
    int 		 		 current_mode_;
    StateMachineState	 state;
    char			 prev_state_machine[50];
    bool			     objectDetected_;
    TagStore* tag_store_;
    Initialization *init_;
    double linear_vel;
    int searchStyle;
    Stoke* stoke_;
    bool bounds;
    geometry_msgs::Pose2D stoke_center_;
    int obstacle_collision_count_;
    int searchCall;
};
#endif
