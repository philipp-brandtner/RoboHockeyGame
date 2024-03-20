#include <ros/ros.h>
#include "puck_finder.h"

int main(int argc, char	**argv)	
{
	// Initiate new ROS node named "puck_finder_node"
	ros::init(argc, argv,	"puck_finder_node");
	// Create new CatchPuck object
  	PuckFinder puckFinder;
  	puckFinder.start();
	ros::spin();
  	return 0;
}
