#include "initialization.h"

#include "TagObject.h"

void Initialization::init(ros::NodeHandle* node_handle, TagStore* tag_store)
{
  node_handle_ = node_handle;
  tag_store_ = tag_store;
  
  rover_publisher_ = node_handle_->advertise<shared_messages::RoverMsg>(("rover_info"), 1, true);
  tag_detect_publisher_ = node_handle_->advertise<shared_messages::TagMsg>(("tag_detect"), 1, true);
  tag_pickup_publisher_ = node_handle_->advertise<std_msgs::Int16>(("tag_pickup"), 1, true);
  debug_publisher_ = node_handle_->advertise<shared_messages::DebugMsg>(("debug_msg"), 1, true);
  
  rover_subscriber_ = node_handle_->subscribe(("rover_info"), 1, &Initialization::roverInfoHandler, this);
  tag_detect_subscriber_ = node_handle_->subscribe(("tag_detect"), 10, &Initialization::tagDetectHandler, this);
  tag_pickup_subscriber_ = node_handle_->subscribe(("tag_pickup"), 10, &Initialization::tagPickupHandler, this);
}

geometry_msgs::Pose2D Initialization::home()
{
  geometry_msgs::Pose2D pos;
  pos.x = 0;
  pos.y = 0;
  pos.theta = 0;
  for (std::set<RoverInfo>::iterator it = rovers_.begin(); it != rovers_.end(); it++)
  {
    pos.x += it->getInitPos().x;
    pos.y += it->getInitPos().y;
  }
  if (rovers_.size() != 0)
  {
    pos.x /= rovers_.size();
    pos.y /= rovers_.size();
  }
  return pos;
}

int Initialization::myRank(std::string name)
{
  geometry_msgs::Pose2D pos;
  RoverInfo rover;
  rover.init(name, pos);
  return std::distance(rovers_.begin(), rovers_.find(rover));
}

void Initialization::publishRoverInfo(shared_messages::RoverMsg msg)
{
  rover_publisher_.publish(msg);
}

void Initialization::publishTagDetect(shared_messages::TagMsg msg)
{
  tag_detect_publisher_.publish(msg);
}

void Initialization::publishTagPickup(std_msgs::Int16 msg)
{
  tag_pickup_publisher_.publish(msg);
}

void Initialization::publishDebug(shared_messages::DebugMsg msg)
{
  debug_publisher_.publish(msg);
}
  
void Initialization::roverInfoHandler(const shared_messages::RoverMsg::ConstPtr& message)
{
  shared_messages::DebugMsg debug_msg;
  debug_msg.name.data = "rover info received";
  debug_msg.debug_string.data = message->name.data;
  //publishDebug(debug_msg);
  std::string name = message->name.data;
  geometry_msgs::Pose2D pos = message->position;
  RoverInfo rover;
  rover.init(name, pos);
  if (rovers_.find(rover) == rovers_.end())
  {
    rovers_.insert(rover);
  }
  else
  {
    rover = *(rovers_.find(rover));
    rovers_.erase(rover);
    rover.update(pos);
    rovers_.insert(rover);
  }
}

void Initialization::tagDetectHandler(const shared_messages::TagMsg::ConstPtr& message)
{
  int id = message->id.data;
  double x = message->pos.x;
  double y = message->pos.y;
  TagObject tag;
  tag.initi(x, y);
  tag_store_->push(id, tag);
}

void Initialization::tagPickupHandler(const std_msgs::Int16::ConstPtr& message)
{
  tag_store_->setUnavailable(message->data);
}

int Initialization::getNumRovers()
{
  return rovers_.size();
}