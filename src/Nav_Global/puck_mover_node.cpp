#include <ros/ros.h>
#include "puck_mover.h"

int main(int argc, char	**argv)	
{
	// Initiate new ROS node named "puck_mover_node"
	ros::init(argc, argv,	"puck_mover_node");
	// Create new CatchPuck object
  	PuckMover puckMover;
  	puckMover.start();
	ros::spin();
  	return 0;
}
