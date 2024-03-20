#ifndef PUCKMOVER_H
#define PUCKMOVER_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>

//service
#include "gruppe6/PuckMoverStart.h"
#include "gruppe6/PuckMoverStop.h"
#include "gruppe6/PuckMoverStatus.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ActionClient;

class PuckMover
{
	public:
		// Constructor
        PuckMover();	
        //start the puckmover and wait for service calls
        void start();

  	private:
  	    //variables
		bool startMoving;
		bool reached;
		geometry_msgs::Point puckPosition;
		
		
  		//declare Node
		ros::NodeHandle node;
		
		//declare action client
		ActionClient actionClient;
		
		//service declarations
		ros::ServiceServer puckMoverStart_service;
		ros::ServiceServer puckMoverStop_service;
		ros::ServiceServer puckMoverStatus_service;
		
		//service calls
		bool moveToPuck_Start(gruppe6::PuckMoverStart::Request  &req, gruppe6::PuckMoverStart::Response &res);
		bool moveToPuck_Stop(gruppe6::PuckMoverStop::Request  &req, gruppe6::PuckMoverStop::Response &res);
		bool moveToPuck_Status(gruppe6::PuckMoverStatus::Request  &req, gruppe6::PuckMoverStatus::Response &res);
};

#endif // PUCKMOVER_H
