#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "gruppe6/PuckMoverStart.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "moverStart_test");

    ros::NodeHandle master;

	ROS_INFO("MOVERT START TEST");
	ros::ServiceClient moverClient = master.serviceClient<gruppe6::PuckMoverStart>("move_To_Puck_Start");
	
    gruppe6::PuckMoverStart moverSrv;
    
    ros::Rate r(1);
	
	// TEST service PuckFinder
	moverSrv.request.puckPosition.z = 1;
	moverSrv.request.puckPosition.x = 2;
	moverSrv.request.puckPosition.y = 0;
	
	ROS_INFO_STREAM("Puck position: \n" << moverSrv.request.puckPosition);

	if(moverClient.call(moverSrv))
	{
		ROS_INFO("Service PuckMover called successfully!");
	}
	else
	{
	   ROS_INFO("Couldn't call service: PuckMoverStart");
	}	
	
	r.sleep();


    return 0;
}
