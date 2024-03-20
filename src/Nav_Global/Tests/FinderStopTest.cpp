#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "gruppe6/PuckFinderStop.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "finderStop_test");

    ros::NodeHandle master;

	ROS_INFO("START MASTER");
	ros::ServiceClient finderClient = master.serviceClient<gruppe6::PuckFinderStop>("find_Puck_Stop");
	
    gruppe6::PuckFinderStop finderSrv;
    
    ros::Rate r(1);

	if(finderClient.call(finderSrv))
	{
		ROS_INFO("Service PuckFinderStop called successfully!");
	}
	else
	{
	   ROS_INFO("Couldn't call service: PuckFinderStop");
	}	
	
	r.sleep();


    return 0;
}
