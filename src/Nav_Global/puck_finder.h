#ifndef PUCKFINDER_H
#define PUCKFINDER_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "gruppe6/Puck.h"
//service
#include "gruppe6/PuckFinderStart.h"
#include "gruppe6/PuckFinderStop.h"
#include "gruppe6/PuckFinderStatus.h"

class PuckFinder
{
	public:
		// Constructor
        PuckFinder();	
        void start();

  	private:
		//indicates if a puck was found
		bool puckFound;
		bool puckAccepted;
		bool stopMoving;
		bool startMoving;
		bool teamColor;
        //init empty point
		geometry_msgs::Point EMPTY_POINT;
		
		//declare Node
		ros::NodeHandle node;
		
		//services
		ros::ServiceServer puckFinderStart_service; 
		ros::ServiceServer puckFinderStop_service; 
		ros::ServiceServer puckFinderStatus_service; 

		//current puck positions
		geometry_msgs::Point yellowPuck_position;
		geometry_msgs::Point bluePuck_position;
		geometry_msgs::Point lastPuck_position;	

		//declare publishers
		ros::Publisher velocity_Publisher;
		//ros::Publisher puckPosition_Publisher;

		//declare subscribers
		ros::Subscriber yellowPuck_subscriber;
		ros::Subscriber bluePuck_subscriber;

		//mover forward
		void move(double velocity_speed, double angular_speed);
		
		//service
		bool findPuck_start(gruppe6::PuckFinderStart::Request  &req, gruppe6::PuckFinderStart::Response &res);
		bool findPuck_stop(gruppe6::PuckFinderStop::Request  &req, gruppe6::PuckFinderStop::Response &res);
		bool findPuck_status(gruppe6::PuckFinderStatus::Request  &req, gruppe6::PuckFinderStatus::Response &res);

		//check the current puck position and move accordingly
        void checkPuckPosition(geometry_msgs::Point yellowPuck_position);

		//get alls found pucks and save them to puck_1, puck_2 and puck_3		
        void getYellowPuckPositions(const gruppe6::Puck& foundPucks);
        //get alls found pucks and save them to puck_1, puck_2 and puck_3
        void getBluePuckPositions(const gruppe6::Puck& foundPucks);        
};

#endif // PUCKFINDER_H
