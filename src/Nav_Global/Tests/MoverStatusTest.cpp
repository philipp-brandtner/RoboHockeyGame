#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "gruppe6/PuckMoverStatus.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "moverStatus_test");

    ros::NodeHandle master;

	ROS_INFO("MOVERT STATUS TEST");
	ros::ServiceClient moverClient = master.serviceClient<gruppe6::PuckMoverStatus>("move_To_Puck_Status");
	
    gruppe6::PuckMoverStatus moverSrv;
    
    ros::Rate r(1);

	if(moverClient.call(moverSrv))
	{
		ROS_INFO("Service PuckMoverStatus called successfully!");
		if(moverSrv.response.reachedGoal)
		{
			ROS_INFO("Reached");
		}
		else
		{
			ROS_INFO("NOT Reached");
		}
	}
	else
	{
	   ROS_INFO("Couldn't call service: PuckMoverStatus!");
	}	
	
	r.sleep();


    return 0;
}
