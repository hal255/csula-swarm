#ifndef INITIALIZATION_H
#define INITIALIZATION_H

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int16.h>
#include <shared_messages/RoverMsg.h>
#include <shared_messages/TagMsg.h>
#include <shared_messages/DebugMsg.h>
#include <sensor_msgs/Image.h>
#include <set>

#include "rover_info.h"
#include "tag_store.h"

class Initialization
{
public:
  void init(ros::NodeHandle* node_handle, TagStore* tag_store);
  geometry_msgs::Pose2D home();
  int myRank(std::string name);
  void publishRoverInfo(shared_messages::RoverMsg rover_msg);
  void publishTagDetect(shared_messages::TagMsg msg);
  void publishTagPickup(std_msgs::Int16 tag_id);
  void publishDebug(shared_messages::DebugMsg msg);
  
  void roverInfoHandler(const shared_messages::RoverMsg::ConstPtr& message);
  void tagDetectHandler(const shared_messages::TagMsg::ConstPtr& messag);
  void tagPickupHandler(const std_msgs::Int16::ConstPtr& message);
  
  int getNumRovers();
  
private:
  ros::NodeHandle *node_handle_;
  TagStore *tag_store_;
  std::set<RoverInfo> rovers_;
  
  ros::Publisher rover_publisher_;
  ros::Publisher tag_detect_publisher_;
  ros::Publisher tag_pickup_publisher_;
  ros::Publisher debug_publisher_;
  
  ros::Subscriber rover_subscriber_;
  ros::Subscriber tag_detect_subscriber_;
  ros::Subscriber tag_pickup_subscriber_;
};

#endif /* INITIALIZATION_H */