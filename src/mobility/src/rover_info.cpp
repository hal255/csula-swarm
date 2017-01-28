#include "rover_info.h"

void RoverInfo::init(std::string name, geometry_msgs::Pose2D pos)
{
  name_ = name;
  init_pos_ = pos;
  curr_pos_ = pos;
}

void RoverInfo::update(geometry_msgs::Pose2D pos)
{
  curr_pos_ = pos;
}

geometry_msgs::Pose2D RoverInfo::getInitPos() const
{
  return init_pos_;
}

geometry_msgs::Pose2D RoverInfo::getCurrPos() const
{
  return curr_pos_;
}
  
bool operator<(const RoverInfo& rover_1, const RoverInfo& rover_2)
{
  return rover_1.name_ < rover_2.name_;
}

bool operator==(const RoverInfo& rover_1, const RoverInfo& rover_2)
{
  return rover_1.name_ == rover_2.name_;
}
