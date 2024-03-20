#include "puck_finder.h"

PuckFinder::PuckFinder()
{
	EMPTY_POINT.x = 0.0;
    EMPTY_POINT.y = 0.0;
    EMPTY_POINT.z = 0.0;
    startMoving = false;
    stopMoving = false;
    puckFound = false;
    puckAccepted = false;
    yellowPuck_position = EMPTY_POINT;
    lastPuck_position = EMPTY_POINT;
    //initialize service
    puckFinderStart_service = node.advertiseService("find_Puck_Start", &PuckFinder::findPuck_start, this);
    puckFinderStop_service = node.advertiseService("find_Puck_Stop", &PuckFinder::findPuck_stop, this);
    puckFinderStatus_service = node.advertiseService("find_Puck_Status", &PuckFinder::findPuck_status, this);
    //Subscribe to puck topics
    yellowPuck_subscriber = node.subscribe("/puck_detection/yellow", 1, &PuckFinder::getYellowPuckPositions, this);
    bluePuck_subscriber = node.subscribe("/puck_detection/blue", 1, &PuckFinder::getBluePuckPositions, this);
    //register the velocity publisher
	velocity_Publisher =node.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);
	//ckPosition_Publisher = node.advertise<geometry_msgs::Point>("Nav_Global/puckPosition", 1000);
}

bool PuckFinder::findPuck_stop(gruppe6::PuckFinderStop::Request  &req, gruppe6::PuckFinderStop::Response &res)
{
	stopMoving = true;
	move(0,0);
	return true;
}


bool PuckFinder::findPuck_status(gruppe6::PuckFinderStatus::Request  &req, gruppe6::PuckFinderStatus::Response &res)
{
	if(stopMoving)
	{
		res.puckPosition = EMPTY_POINT;
	}
	else
	{
		res.puckPosition = lastPuck_position;
	}
	return true;
}

bool PuckFinder::findPuck_start(gruppe6::PuckFinderStart::Request  &req, gruppe6::PuckFinderStart::Response &res)
{
	teamColor = req.teamColor;
	startMoving = true;	
	return true;
}

void PuckFinder::start()
{
	ROS_INFO("START TO SEARCH PUCK");
	while(ros::ok())
	{		
		if(startMoving)
		{
			ros::Rate rate(10);
			startMoving = false;			
			puckFound = false;
			puckAccepted = false;
			stopMoving = false;
			lastPuck_position = EMPTY_POINT;
			while(!puckAccepted && !stopMoving)
			{	
				if(teamColor)
				{
					checkPuckPosition(yellowPuck_position);				
				}
				else
				{
					checkPuckPosition(bluePuck_position);
				}
				rate.sleep();
			}	
			
			if(!stopMoving)
			{
				ROS_INFO_STREAM("PUCK FOUND");
			}
			else
			{
				startMoving = false;
				stopMoving = false;
				ROS_INFO_STREAM("STOPED MOVING");		
			}			
		}
		ros::spinOnce();
	}
}


void PuckFinder::getYellowPuckPositions(const gruppe6::Puck &foundPucks)
{
    float_t puck1 = foundPucks.puck_1.z;
    float_t puck2 = foundPucks.puck_2.z;
    float_t puck3 = foundPucks.puck_3.z;

	switch(foundPucks.count)
	{
		case 1:	
			yellowPuck_position = foundPucks.puck_1;
			break;
		case 2:
			if(puck1 < puck2)
			{
				yellowPuck_position = foundPucks.puck_1;
			}
			else
			{
				yellowPuck_position = foundPucks.puck_2;
			}
			break;

		case 3:
			if(puck1 < puck2 && puck1 < puck3 )
			{
				yellowPuck_position = foundPucks.puck_1;
			}
			else if(puck2 < puck1 && puck2 < puck3)
			{
				yellowPuck_position = foundPucks.puck_2;
			}
			else
			{
				yellowPuck_position = foundPucks.puck_3;
			}
			break;

		default:
			yellowPuck_position = EMPTY_POINT;
			break;
	}	
}

void PuckFinder::getBluePuckPositions(const gruppe6::Puck &foundPucks)
{
    float_t puck1 = foundPucks.puck_1.z;
    float_t puck2 = foundPucks.puck_2.z;
    float_t puck3 = foundPucks.puck_3.z;

	switch(foundPucks.count)
	{
		case 1:	
			bluePuck_position = foundPucks.puck_1;
			break;
		case 2:
			if(puck1 < puck2)
			{
				bluePuck_position = foundPucks.puck_1;
			}
			else
			{
				bluePuck_position = foundPucks.puck_2;
			}
			break;

		case 3:
			if(puck1 < puck2 && puck1 < puck3 )
			{
				bluePuck_position = foundPucks.puck_1;
			}
			else if(puck2 < puck1 && puck2 < puck3)
			{
				bluePuck_position = foundPucks.puck_2;
			}
			else
			{
				bluePuck_position = foundPucks.puck_3;
			}
			break;

		default:
			bluePuck_position = EMPTY_POINT;
			break;
	}	
}


void PuckFinder::checkPuckPosition(geometry_msgs::Point currentPuck_position)
{	
	double vel_speed = 0.0;
	double angular_speed = 0.0;
	
	ROS_INFO_STREAM("Puck Position Z: " << currentPuck_position.z);
	ROS_INFO_STREAM("Puck Position X: " << currentPuck_position.x);
	
    if(!puckFound)
	{
		if(currentPuck_position.z > 0)
		{
			puckFound = true;
			
			
			angular_speed = 0.0;	
			lastPuck_position = currentPuck_position;
		}
		else
		{
			ROS_INFO("NO PUCK Turn Right");
			angular_speed = -0.4;
		}
	}
	else
	{
		puckAccepted = true;
	}
/*	else
	{		
		if(currentPuck_position.z == lastPuck_position.z)
		{
			ROS_INFO("Found True");
			puckAccepted = true;	
		}
		else
		{
			ROS_INFO("Found False");
			puckFound = false;
		}        
    }
    */ 

	move(vel_speed, angular_speed);	
}


void PuckFinder::move(double velocity_speed, double angular_speed)
{
	//declare a Twist message to send velocity commands
	geometry_msgs::Twist VelocityMessage;

	//setup linear velocity
	VelocityMessage.linear.x = velocity_speed;
	VelocityMessage.linear.y =0;
	VelocityMessage.linear.z =0;
	//setup angular velocity
	VelocityMessage.angular.x = 0;
	VelocityMessage.angular.y = 0;
	VelocityMessage.angular.z =angular_speed;

	velocity_Publisher.publish(VelocityMessage);
	ros::spinOnce();
}








