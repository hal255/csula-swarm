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
#ifndef ROVER_H
#define ROVER_H

#include <math.h>
#include <ros/ros.h>

#include <geometry_msgs/Pose2D.h>
 #include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <angles/angles.h>
#include <string>
#include <TagObject.h>
#include <shared_messages/TagsImage.h>
struct Publishers;

class Rover
{

private:
  geometry_msgs::Pose2D    position_;
  geometry_msgs::Pose2D    goal_position_;
  TagObject                targets_collected_[256];
  bool                     hasTag_;
  TagObject*               tag_;
  std::string              hostname_;
  ros::Publisher*          velocityPublish_;

public:
  Rover();
  Rover(ros::Publisher* velocityPublish);
  // Rover(geometry_msgs::Pose2D start_position, ros::Publisher& velocityPublish);
  ~Rover();

  //void setPublisher(ros::Publisher& velocityPublish);
  //ros::Publisher& getPublisher();

  void setHostName(std::string hostname);
  std::string getName();
  bool obstacleDetected();
  bool tagDetected(const shared_messages::TagsImage::ConstPtr& message);
  bool isTagCollected(const shared_messages::TagsImage::ConstPtr& message);
  void setRoverPosition(geometry_msgs::Pose2D position);
  geometry_msgs::Pose2D& getPosition();
  void setGoalPosition(geometry_msgs::Pose2D);
  geometry_msgs::Pose2D& getGoalPosition();
  void rotate(double angle);
  void rotate(double angle, double epsilon);
  void rotate(double angle, double epsilon, double angular_velocity);
  void faceGoal();
  double getAngle();
  void translate(double linear_velocity, double angular_velocity);
  bool isAtGoal();
  bool isAtHome();
  void setTagCollected(const shared_messages::TagsImage::ConstPtr& message);
  void setTagCollected(const std_msgs::Int16::ConstPtr& message);
  void setGoalHome();
  bool hasTag();
  // std_msgs::Int16 getTagInfo();
  void pickUpTag(const shared_messages::TagsImage::ConstPtr& message);
  void dropTag();
  void setVelPublisher(Publishers*);
  std_msgs::Int16 target_detected_;
};
#endif



