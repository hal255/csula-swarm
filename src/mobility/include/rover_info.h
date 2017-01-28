#ifndef ROVER_INFO_H
#define ROVER_INFO_H

#include <string>
#include <geometry_msgs/Pose2D.h>

class RoverInfo
{
public:
  void init(std::string name, geometry_msgs::Pose2D pos);
  void update(geometry_msgs::Pose2D pos);
  
  geometry_msgs::Pose2D getInitPos() const;
  geometry_msgs::Pose2D getCurrPos() const;
  
  friend bool operator<(const RoverInfo& rover_1, const RoverInfo& rover_2);
  friend bool operator==(const RoverInfo& rover_1, const RoverInfo& rover_2);

private:
  std::string name_;
  geometry_msgs::Pose2D init_pos_;
  geometry_msgs::Pose2D curr_pos_;
};

#endif /* ROVER_INFO_H */