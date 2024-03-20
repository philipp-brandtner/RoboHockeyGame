#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "gruppe6/PuckFinderStatus.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "finderStatus_test");

    ros::NodeHandle master;

	ROS_INFO("START MASTER");
	ros::ServiceClient finderClient = master.serviceClient<gruppe6::PuckFinderStatus>("find_Puck_Status");
	
    gruppe6::PuckFinderStatus finderSrv;
    
    ros::Rate r(1);

	if(finderClient.call(finderSrv))
	{
		ROS_INFO_STREAM("Service PuckFinderStop called successfully!\n" << finderSrv.response.puckPosition);
		
	}
	else
	{
	   ROS_INFO("Couldn't call service: PuckFinderStop");
	}	
	
	r.sleep();


    return 0;
}
