#include "Stoke.h"
#include <geometry_msgs/Pose2D.h>
#include <string>

void Stoke::computeAngle(geometry_msgs::Pose2D gLoc){
	goalPosition.theta = gLoc.theta;
	// min = goalPosition.theta;
	// max = goalPosition.theta + 120 * (M_PI/180);
	goalPosition.x = 0.5 * cos(goalPosition.theta);
	goalPosition.y = 0.5 * sin(goalPosition.theta);
}

geometry_msgs::Pose2D Stoke::getPosition(){ 
	//search from center
		goalPosition.x += 0.2 * cos(goalPosition.theta);
		goalPosition.y += 0.2 * sin(goalPosition.theta);
	return goalPosition;                
}

void Stoke::incrementAngle(){
		// increase angle by 10 degrees
		goalPosition.theta += 10 * M_PI/180;
}

void Stoke::setPosition(geometry_msgs::Pose2D pos){
	currentPosition = pos;
}