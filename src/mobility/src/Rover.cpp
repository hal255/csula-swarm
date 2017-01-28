/****************************************************
 * Rover.cpp                                        *
 *                                                  *
 *  Created on: Feb 12, 2016                        *
 *   Edited on: Feb 22, 2016                        *
 *      Author: Jonathan Sahagun                    *
 *      Author: Abner Hernandez                     *
 *                                                  *
 *      Version: 0.2                                *
 ****************************************************
*/

#include "Rover.h"
#include "TagObject.h"
#include <ros/ros.h>
#include <math.h>

#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include <ros/ros.h>
#include <shared_messages/TagsImage.h>
#include "Publishers.h"
Rover::Rover()
{
}

Rover::Rover(ros::Publisher* velocityPublish)
{
  velocityPublish_ = velocityPublish;
  hasTag_ = false;
  // isDetected = false;
}

Rover::~Rover()
{
}

void Rover::setVelPublisher(Publishers* pubs)
{
  velocityPublish_ = &pubs->velocityPublish;
}



bool Rover::isTagCollected(const shared_messages::TagsImage::ConstPtr& message)
{
  int targDet = message->tags.data[0];
  tag_ = &targets_collected_[targDet];
  return tag_->isTagCollected();
}

void Rover::setHostName(std::string hostname)
{
  hostname_ = hostname;
}
  
std::string Rover::getName()
{
  return hostname_;
}
  
void Rover::setRoverPosition(geometry_msgs::Pose2D position)
{
  position_ = position;
}


geometry_msgs::Pose2D& Rover::getPosition()
{
  return position_;
}


void Rover::setGoalPosition(geometry_msgs::Pose2D position)
{
  goal_position_ = position;
}
  

geometry_msgs::Pose2D& Rover::getGoalPosition()
{
  return goal_position_;
}
  

void Rover::rotate(double angle)
{
  // If not defined default epsilon is 0.1 which is approximately 5 degrees
  rotate(angle, 0.1);
}
  

void Rover::rotate(double angle, double epsilon)
{
    // If not defined default angular_speed is 0.2
    rotate(angle, epsilon, 0.2);
}

void Rover::rotate(double angle, double epsilon, double angular_speed)
{
  
  // Adds the angle of the position to the one of the angle
  double goal_theta = position_.theta + angle;
  
  // Get the angle difference between the goal and the cirrent position
  double difference = angles::shortest_angular_distance(position_.theta, goal_theta);
  
  // If the difference is greater than the threshold we have assigned
  if (difference > epsilon) 
  {
     //rotate left continuously
    translate(0.0, angular_speed);
    
  }
  // If the difference is less than negative epsilon
  else if (difference < (epsilon * -1))
  {
    //rotate right continuously
    translate(0.0, -1.0 * angular_speed); 
  }
  else
  {
    return;
  }

  // This will continouslly loop until our epsilon is fixed
  while(fabs(difference) > epsilon)
  {
    difference = angles::shortest_angular_distance(position_.theta, goal_theta);
  }
  
  translate(0.0, 0.0); // stop rotating
}

void Rover::faceGoal()
{
  double delta_x = position_.x - goal_position_.x;
  double delta_y = position_.y - goal_position_.y;

  // The angle between the two points
  double angle = atan2(delta_y, delta_x);

  // The angle difference between the angle between the two points and the angle the rover is facing
  double delta_angle = angles::shortest_angular_distance(position_.theta, angle);

  // Rotate to face the goal
  rotate(delta_angle);
}


double Rover::getAngle()
{
  return position_.theta;
}



void Rover::translate(double linear_velocity, double angular_velocity)
{
  geometry_msgs::Twist velocity;
  // Copied from mobility
  velocity.linear.x = linear_velocity * 1.5;
  //scaling factor for sim; removed by aBridge node
  velocity.angular.z = angular_velocity * 8;
  velocityPublish_->publish(velocity);
}
  
bool Rover::isAtGoal()
{
  // Some random threshold to compare to. May change later
  double epsilon = 1.0;
  // The X difference between the current X position and X goal position
  double delta_x = position_.x - goal_position_.x;
  // The Y difference between the current Y position and Y goal position
  double delta_y = position_.y - goal_position_.y;
  
  // If the X and Y coodrdinates is relatively close to the threshold we return true
  if (fabs(delta_y) < epsilon && fabs(delta_x) < epsilon)
  {
    return true;
  }
  // Else we are close enough to our goal so we return false
  else
  {
    return false;
  }
  
}

bool Rover::isAtHome()
{
  /* 
  * TODO : Some random threshold to compare to. May change later depending on the size
  * of the home
  */
  double epsilon = 1.0;
  
  /* 
  * Absolute value of the current X & Y coordinates is less than epsilon
  * Then we are inside of our threshold and this returns true
  */
  if (fabs(position_.x) < epsilon && fabs(position_.y) < epsilon)
  {
    return true;
  }
  
  // If the X & Y coordinates are outside the threshold, we will not consider it as at home
  else
  {
    return false;
  }
  
}

void Rover::setGoalHome()
{
  //Set angle to center as goal heading
  goal_position_.theta = M_PI + atan2(position_.y, position_.x);
  
  /*
  * Set center as goal position so that the 
  * rover will have its goal to go home
  */
  goal_position_.x = 0.0;
  goal_position_.y = 0.0;
  
}
  
bool Rover::hasTag()
{
  return hasTag_;
}


void Rover::setTagCollected(const shared_messages::TagsImage::ConstPtr& message){
  tag_ = &targets_collected_[message->tags.data[0]];
  tag_->setTagCollected();
}

void Rover::setTagCollected(const std_msgs::Int16::ConstPtr& message){
  tag_ = &targets_collected_[message->data];
  tag_->setTagCollected();
}

void Rover::pickUpTag(const shared_messages::TagsImage::ConstPtr& message)
{
  tag_ = &targets_collected_[message->tags.data[0]];
  tag_->setTagCollected();
  hasTag_ = true;
}

void Rover::dropTag()
{
  hasTag_ = false;
}
