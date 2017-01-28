#include "MobilityHelper.h"

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
#include <shared_messages/TagMsg.h>
#include <shared_messages/RoverMsg.h>


// To handle shutdown signals so the node quits properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

#include "Rover.h"
#include "Publishers.h"
#include "Subscribers.h"
#include "tag_store.h"
#include "initialization.h"
#include "rover_info.h"
#include "Stoke.h"
#include "Timers.h"
#include <iostream>

#include <math.h>

MobilityHelper::MobilityHelper(Rover* rover, ros::NodeHandle* nh)
{

    rover_ = rover;
    pubs = new Publishers;
    subs = new Subscribers;
    timers = new Timers;
    state = TRANSFORM;
    rover->setVelPublisher(pubs);
    objectDetected_ = false;
    tag_store_ = new TagStore();
    init_ = new Initialization();
    init_->init(nh, tag_store_);
    linear_vel = -0.3;
    searchStyle = 0;
    stoke_ = new Stoke;
    stoke_center_ = rover->getGoalPosition();
    obstacle_collision_count_ = 0;
    searchCall = 0;

}

void MobilityHelper::search(const ros::TimerEvent&)
{
    random_numbers::RandomNumberGenerator rng;
    std_msgs::String stateMachineMsg;
    geometry_msgs::Pose2D currentLocation = rover_->getPosition();
    geometry_msgs::Pose2D goalLocation = rover_->getGoalPosition();
    if(current_mode_ == 2 || current_mode_ == 3)
    {
        switch(state)
        {
            case TRANSFORM:
            {
                stateMachineMsg.data = "TRANSFORMING";

                //If angle between current and goal is significant
                if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.1) {
                    state = ROTATE; //rotate
                }

                //If goal has not yet been reached
                else if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {
                    state = TRANSLATE; //translate
                }

                //If returning with a target
                else if (rover_->hasTag()) {
                //If goal has not yet been reached
                    if (hypot(0.0 - currentLocation.x, 0.0 - currentLocation.y) > 0.5) {
                        //set angle to center as goal heading
                        goalLocation.theta = M_PI +  atan2(currentLocation.y, currentLocation.x);
                        //set center as goal position
                        goalLocation.x = 0;
                        goalLocation.y = 0;
                    }
                    //Otherwise, reset target and select new random uniform heading
                    else {
                        rover_->dropTag();
                        if (tag_store_->tagAvailable())
                        {
                          goalLocation =stoke_->getPosition();                       
                        }
                    }
                }
                else if(tag_store_->tagAvailable()){
                    TagObject tag = tag_store_->pop(currentLocation);
                    goalLocation.x = tag.getX();
                    goalLocation.y = tag.getY();
                    goalLocation.theta = atan2(goalLocation.y, goalLocation.x);
                    double hyp = sqrt(pow(currentLocation.x - goalLocation.x, 2) + pow(currentLocation.y - goalLocation.y, 2));
                    if(hyp <= 3){
                        state = ROTATE;
                        goalLocation.theta += .2 + currentLocation.theta;
                    }
                }
                //Otherwise, assign a new goal
                else if(searchStyle == 0) {
                    if(searchCall != 0){
                        goalLocation = stoke_->getPosition();
                    }
                    else{
                        goalLocation.theta = currentLocation.theta + M_PI;
                        goalLocation.x = currentLocation.x + 1.5 * cos(goalLocation.theta);
                        goalLocation.y = currentLocation.y + 1.5 * sin(goalLocation.theta);
                        stoke_->computeAngle(goalLocation);
                        ++searchCall;
                    }
                    
                } 
                else if(searchStyle == 1){
                    // scale about 70 percent of distance from the wall to center
                    double scaleFactor = 7.0/10.0;
                    // total distance from rover to center
                    double hyp = sqrt(pow(currentLocation.x, 2) + pow(currentLocation.y, 2));
                    // angle to the center
                    goalLocation.theta = M_PI + atan2(currentLocation.y, currentLocation.x);
                    // goalLocation.x = currentLocation.x + abs(currentLocation.x * scaleFactor) * cos(goalLocation.theta);
                    // goalLocation.y = currentLocation.y + abs(currentLocation.y * scaleFactor) * sin(goalLocation.theta);
                    // set goal distance 7/10 th meters from current distance
                    goalLocation.x = currentLocation.x + hyp * scaleFactor * cos(goalLocation.theta);
                    goalLocation.y = currentLocation.y + hyp * scaleFactor * sin(goalLocation.theta);
                    searchStyle = 0;
                }
                else{
                    goalLocation.theta = M_PI + atan2(currentLocation.y, currentLocation.x);
                    goalLocation.x = 0;
                    goalLocation.y = 0;
                }
                //Purposefully fall through to next case without breaking
            }

            //Calculate angle between currentLocation.theta and goalLocation.theta
            //Rotate left or right depending on sign of angle
            //Stay in this state until angle is minimized
            case ROTATE: {
                stateMachineMsg.data = "ROTATING";
                if (angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta) > 0.1) {
                    if(!objectDetected_)
                        rover_->translate(0.0, 0.2); //rotate left
                    else
                        rover_->translate(0.0, 0.2); //rotate left
                }
                else if (angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta) < -0.1) {
                    if(!objectDetected_)
                        rover_->translate(0.0, -0.2); //rotate left
                    else
                        rover_->translate(0.0, -0.2); //rotate left
                }
                else {
                    rover_->translate(0.0, 0.0); //stop
                    state = TRANSLATE; //move to translate step
                }
                break;
            }

            case TRANSLATE: {
                stateMachineMsg.data = "TRANSLATING";
                if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {
                    rover_->translate(0.2, 0.0);
                }
                else {
                    rover_->translate(0.0, 0.0); //stop
                    state = TRANSFORM; //move back to transform step
                }
                break;
            }

            default: {
                break;
            }
        }
    }

    else { // mode is NOT auto
        // publish current state for the operator to see
        stateMachineMsg.data = "WAITING";
    }
    rover_->setRoverPosition(currentLocation);
    rover_->setGoalPosition(goalLocation);
    // publish state machine string for user, only if it has changed, though
    if (strcmp(stateMachineMsg.data.c_str(), prev_state_machine) != 0) {
        pubs->stateMachinePublish.publish(stateMachineMsg);
        sprintf(prev_state_machine, "%s", stateMachineMsg.data.c_str());
    }
}


void MobilityHelper::joyCmdHandler(const geometry_msgs::Twist::ConstPtr& message)
{
    if(current_mode_ == 0 || current_mode_ == 1){
        rover_->translate(message->linear.x, message->angular.z);
    }

}

void MobilityHelper::modeHandler(const std_msgs::UInt8::ConstPtr& message)
{
    current_mode_ = message->data;
    rover_->translate(0.0, 0.0);
}

void MobilityHelper::targetHandler(const shared_messages::TagsImage::ConstPtr& message)
{
    //if this is the goal target
    if (message->tags.data[0] == 256)
    { 
        // geometry_msgs::Pose2D currentPosition = rover_->getPosition();
        // currentPosition.x = 0;
        // currentPosition.y = 0;
        //rover_->setRoverPosition(currentPosition);
        searchStyle = 0;
        //if we were returning with a target
        if (rover_->hasTag())
        {
            //publish to scoring code
            pubs->targetDropOffPublish.publish(message->image);
            rover_->dropTag();

            //rover_->setGoalPosition(gLoc);
            //set the rover back to patrol
        }
    }

    if (!rover_->hasTag() && message->tags.data[0] != 256 && tag_store_->isTagAvailable(message->tags.data[0]))
    {       
            //head to home base.
            searchStyle = 2;
            std_msgs::Int16 targetDetected;
            targetDetected.data = message->tags.data[0];
            tag_store_->setUnavailable(targetDetected.data);
            //set angle to center as goal heading
            geometry_msgs::Pose2D gLoc;
            gLoc.theta = M_PI + atan2(rover_->getPosition().y, rover_->getPosition().x);
            gLoc.x = 0;
            gLoc.y = 0;
            rover_->setGoalPosition(gLoc);
            rover_->pickUpTag(message);
            //publish detected target
            pubs->targetsCollectedPublish.publish(targetDetected);
            shared_messages::TagMsg msg;
            msg.id.data = message->tags.data[0];
            msg.pos = rover_->getPosition();
            init_->publishTagDetect(msg);
            //publish to scoring code
            pubs->targetPickUpPublish.publish(message->image);
            std_msgs::Int16 id;
            id.data = message->tags.data[0];
            init_->publishTagPickup(id);
            //switch to transform state to trigger return to center
            state = TRANSFORM;
    }
    else if(message->tags.data[0] != 256 && tag_store_->isTagAvailable(message->tags.data[0])){
            //publish to scoring code
             // pubs->targetPickUpPublish.publish(message->image);
              std_msgs::Int16 id;
              id.data = message->tags.data[0];
              geometry_msgs::Pose2D currentLocation = rover_->getPosition();
              TagObject tag;
              tag.initi(currentLocation.x, currentLocation.y);
              tag.setTagDetected();
              tag_store_->push(message->tags.data[0], tag);
              shared_messages::TagMsg msg;
              msg.id;
              id.data = message->tags.data[0];
              msg.pos = rover_->getPosition();
              //init_->publishTagDetect(msg);
    }


}

void MobilityHelper::obstacleHandler(const std_msgs::UInt8::ConstPtr& message)
{
    if (message->data > 0 && obstacle_collision_count_ > 1) {
        geometry_msgs::Pose2D currentLocation = rover_->getPosition();
        geometry_msgs::Pose2D goalLocation = rover_->getPosition();

        //select new heading 0.2 radians to the left
        if (message->data == 1 || message->data == 2) {
            goalLocation.theta = currentLocation.theta + 0.2;
        }
        if(currentLocation.x < -5 || currentLocation.x > 5){
            //set the rover for the stoke home. 
            searchStyle = 1;
            if(!objectDetected_){
            stoke_->incrementAngle();
            }
        }
        else if(currentLocation.y < -5 || currentLocation.y > 5){
            // set the rover to head home
            searchStyle = 1;
            if(!objectDetected_){
            stoke_->incrementAngle();
            }
        }
        //select new position 50 cm from current location
        goalLocation.x = currentLocation.x + (1.0 * cos(goalLocation.theta));
        goalLocation.y = currentLocation.y + (1.0 * sin(goalLocation.theta));

        rover_->setGoalPosition(goalLocation);
        //switch to transform state to trigger collision avoidance
        state = TRANSFORM;
        objectDetected_ = true;
    }
    else if(obstacle_collision_count_ < 2){
        ++obstacle_collision_count_;
    }
    else{
        obstacle_collision_count_ = 0;
        objectDetected_ = false;
    }
}

void MobilityHelper::odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
    geometry_msgs::Pose2D currentLocation;
    //Get (x,y) location directly from pose
    currentLocation.x = message->pose.pose.position.x;
    currentLocation.y = message->pose.pose.position.y;
    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    currentLocation.theta = yaw;
    rover_->setRoverPosition(currentLocation);
}

void MobilityHelper::publishStatusTimerEventHandler(const ros::TimerEvent&)
{
    std_msgs::String msg;
    msg.data = "online";
    pubs->status_publisher.publish(msg);
    
    shared_messages::RoverMsg rover_msg;
    rover_msg.name.data = rover_->getName();
    rover_msg.position = rover_->getPosition();
    init_->publishRoverInfo(rover_msg);
    shared_messages::RoverMsg home_msg;
    home_msg.name.data = "home";
    home_msg.position = init_->home();
    init_->publishRoverInfo(home_msg);
    shared_messages::DebugMsg num;
    num.name.data = "rove index";
    num.debug_int.data = init_->myRank(rover_->getName());
    num.debug_string.data = rover_->getName();
    init_->publishDebug(num);
}

// Safety precaution. No movement commands - might have lost contact with ROS. Stop the rover.
// Also might no longer be receiving manual movement commands so stop the rover.
void MobilityHelper::killSwitchTimerEventHandler(const ros::TimerEvent& t)
{
    // No movement commands for killSwitchTime seconds so stop the rover
    rover_->translate(0.0, 0.0);
    double current_time = ros::Time::now().toSec();
    ROS_INFO("In mobility.cpp:: killSwitchTimerEventHander(): Movement input timeout. Stopping the rover at %6.4f.", current_time);
}

void MobilityHelper::targetsCollectedHandler(const std_msgs::Int16::ConstPtr& message) {
    tag_store_->collected(message->data);
    // targetsCollected[message->data] = 1;
}
