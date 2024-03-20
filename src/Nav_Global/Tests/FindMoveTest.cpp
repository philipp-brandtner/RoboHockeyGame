#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "gruppe6/PuckFinderStart.h"
#include "gruppe6/PuckFinderStatus.h"
#include "gruppe6/PuckMoverStart.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "finder_test");

    ros::NodeHandle master;

	ROS_INFO("START MASTER");
	ros::ServiceClient finderStartClient = master.serviceClient<gruppe6::PuckFinderStart>("find_Puck_Start");
	ros::ServiceClient finderStatusClient = master.serviceClient<gruppe6::PuckFinderStatus>("find_Puck_Status");
	ros::ServiceClient moverStartClient = master.serviceClient<gruppe6::PuckMoverStart>("move_To_Puck_Start");
	
    gruppe6::PuckFinderStart finderStartSrv;
    gruppe6::PuckFinderStatus finderStatusSrv;
    gruppe6::PuckMoverStart moverStartSrv;
    
    ros::Rate r(10);

	// TEST service PuckFinder
	finderStartSrv.request.teamColor = 0;
	
	if(finderStartClient.call(finderStartSrv))
	{
		ROS_INFO("Service PuckFinder called successfully!");
	}
	else
	{
	   ROS_INFO("Couldn't call service: PuckFinder (MASTER_NODE)");
	}	
	
	bool out = false;
	while(!out)
	{
		if(finderStatusClient.call(finderStatusSrv))
		{
			//ROS_INFO("Service PuckFinder called successfully!");
			if(finderStatusSrv.response.puckPosition.z > 0)
			{
				ROS_INFO_STREAM("Point to move: \n" << finderStatusSrv.response.puckPosition);
				out = true;
			}
		}
		else
		{
		   ROS_INFO("Couldn't call service: PuckFinder (MASTER_NODE)");
		}
		r.sleep();	
	}	
	
	moverStartSrv.request.puckPosition = finderStatusSrv.response.puckPosition;
	
	if(moverStartClient.call(moverStartSrv))
	{
		ROS_INFO("Service PuckMover called successfully!");
	}
	else
	{
	   ROS_INFO("Couldn't call service: PuckMoverStart");
	}	
	
	
	
    return 0;
}
