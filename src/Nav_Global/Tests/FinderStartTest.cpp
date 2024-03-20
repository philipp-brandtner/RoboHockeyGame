#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "gruppe6/PuckFinderStart.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "finder_test");

    ros::NodeHandle master;

	ROS_INFO("START MASTER");
	ros::ServiceClient finderClient = master.serviceClient<gruppe6::PuckFinderStart>("find_Puck_Start");
	
    gruppe6::PuckFinderStart finderSrv;
    
    ros::Rate r(1);


	// TEST service PuckFinder
	finderSrv.request.teamColor = 0;

	if(finderClient.call(finderSrv))
	{
		ROS_INFO("Service PuckFinder called successfully!");
	}
	else
	{
	   ROS_INFO("Couldn't call service: PuckFinder (MASTER_NODE)");
	}	
	
	r.sleep();


    return 0;
}
