#ifndef MOBILITYSTATEMACHINE_H
#define  MOBILITYSTATEMACHINE_H

namespace std_msgs{
	class Int16;
}

namespace geometry_msgs{
	class Pose2D;
}

using std_msgs::Int16;
using geometry_msgs::Pose2D;

class  MobilityStateMachine{
public:
	MobilityStateMachine(Pose2D&, Pose2D&, Int16&);
	void goHome(void);
	void goPose(int x, int y);
	void rotate(int degress);
	
	voidwait(int time);
private:
	Pose2D* currentLocation;
	Pose2D* goalLocation;
	Int16* targetDetected;
};


#endif
