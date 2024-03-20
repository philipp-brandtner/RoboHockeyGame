#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "gruppe6/PuckMoverStop.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "moverStop_test");

    ros::NodeHandle master;

	ROS_INFO("MOVERT START TEST");
	ros::ServiceClient moverClient = master.serviceClient<gruppe6::PuckMoverStop>("move_To_Puck_Stop");
	
    gruppe6::PuckMoverStop moverSrv;
    
    ros::Rate r(1);

	if(moverClient.call(moverSrv))
	{
		ROS_INFO("Service PuckMoverStop called successfully!");
		if(moverSrv.response.stopped)
		{
			ROS_INFO_STREAM("STOPPED");
		}
		else
		{
			ROS_INFO("NOT STOPPED");
		}
	}
	else
	{
	   ROS_INFO("Couldn't call service: PuckMoverStop!");
	}	
	
	r.sleep();


    return 0;
}
