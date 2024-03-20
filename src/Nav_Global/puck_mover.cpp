#include "puck_mover.h"

PuckMover::PuckMover() : actionClient("move_base", true)
{
	//initialize variables
	startMoving = false;
	reached = false;
	puckPosition.x = 0;
	puckPosition.y = 0;
	puckPosition.z = 0;
	//initialize service
    puckMoverStart_service = node.advertiseService("move_To_Puck_Start", &PuckMover::moveToPuck_Start, this);
    puckMoverStop_service = node.advertiseService("move_To_Puck_Stop", &PuckMover::moveToPuck_Stop, this);
    puckMoverStatus_service = node.advertiseService("move_To_Puck_Status", &PuckMover::moveToPuck_Status, this);
}

//This service sets a flag in order to start the moving from start function
bool PuckMover::moveToPuck_Start(gruppe6::PuckMoverStart::Request  &req, gruppe6::PuckMoverStart::Response &res)
{
	ROS_INFO_STREAM("GOT START SERVICE CALL\n" << req.puckPosition);
	startMoving = true;
	puckPosition = req.puckPosition;
	return true;
}

//This service cancel all goals currently at the action client
bool PuckMover::moveToPuck_Stop(gruppe6::PuckMoverStop::Request  &req, gruppe6::PuckMoverStop::Response &res)
{
	ROS_INFO("GOT STOP SERVICE CALL");
	if(startMoving)
	{
		actionClient.stopTrackingGoal();
		actionClient.cancelAllGoals();
	}
	res.stopped = true;
	return true;
}

//This service return true if goal is reached and false if not reached
bool PuckMover::moveToPuck_Status(gruppe6::PuckMoverStatus::Request  &req, gruppe6::PuckMoverStatus::Response &res)
{
	ROS_INFO("GOT STATUS SERVICE CALL");
	res.reachedGoal = reached;   
	return true;
}


//This function waits for the "move_To_Puck_Start" service call and moves to the point given by it. Afte that, it waits again.
//The active action can be canceld by the "move_To_Puck_Stop" service and the status can be seen from "move_To_Puck_Status" service 
void PuckMover::start()
{
	while(ros::ok())
	{
		//ROS_INFO_STREAM("Start moving: " << startMoving);
		if(startMoving)
		{
			ROS_INFO("STARTED");
			reached = false;
			
			ROS_INFO("Waiting for action server to start.");
			actionClient.waitForServer();
			ROS_INFO("Action server started, sending goal.");
			
			
			 //wait for the action server to come up
			while(!actionClient.waitForServer(ros::Duration(5.0)))
			{
				ROS_INFO("Waiting for the move_base action server to come up");
			}

			move_base_msgs::MoveBaseGoal MoveBaseGoal;

			//Setup goal for the robot
			MoveBaseGoal.target_pose.header.frame_id = "base_link";
			MoveBaseGoal.target_pose.header.stamp = ros::Time::now();

			MoveBaseGoal.target_pose.pose.position.x = puckPosition.z;
			MoveBaseGoal.target_pose.pose.position.y = puckPosition.x;
			//MoveBaseGoal.target_pose.pose.orientation.w = 0.0;
			
			//Callculate quaternions from angular
			tf::Quaternion quaternion;
			quaternion = tf::createQuaternionFromYaw(0);
			
			geometry_msgs::Quaternion qMsg;
			tf::quaternionTFToMsg(quaternion, qMsg);
			
			MoveBaseGoal.target_pose.pose.orientation = qMsg;

					
			ROS_INFO("Sending goal");
			actionClient.sendGoal(MoveBaseGoal);
			
			ROS_INFO("Try to reach Goal");
			actionClient.waitForResult();

			if(actionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				reached = true;
				ROS_INFO("GOAL REACHED");
			}
			else
			{			
				reached = false;
				ROS_INFO("GOAL NOT REACHED");
			}	
			
			puckPosition.x = 0;
			puckPosition.y = 0;
			puckPosition.z = 0;		
			
			startMoving = false;	
		}
		ros::spinOnce();
	}	
}

