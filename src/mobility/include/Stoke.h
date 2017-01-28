#ifndef STOKE_H
#define STOKE_H


#include <geometry_msgs/Pose2D.h>
#include <string>
class Stoke{
public:
    void computeAngle(geometry_msgs::Pose2D);
    void incrementAngle();
    void setPosition(geometry_msgs::Pose2D);
    geometry_msgs::Pose2D getPosition();
private:
    void move();
    geometry_msgs::Pose2D goalPosition;
    geometry_msgs::Pose2D currentPosition;
    double min;
    double max;
};

    

#endif