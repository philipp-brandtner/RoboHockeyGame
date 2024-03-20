#ifndef PUCKMOVER_H
#define PUCKMOVER_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "gruppe6/Puck.h"
#include "gruppe6/Pole.h"
#include "std_msgs/Bool.h"
#include <list>

class PuckMover 
{
	public:
		// Constructor
        PuckMover();
		void startMoving();		

  	private:
		//indicates, if the robot has reached the position in front of his located puck
		bool positionReached;
		bool puckFound;
		bool startStop_PuckMover;

        float_t poleDistance;

        //init empty point
		geometry_msgs::Point EMPTY_POINT;

		//current puck positions
		geometry_msgs::Point yellowPuck_position;
        geometry_msgs::Point last_yellowPuck_position;
		geometry_msgs::Point bluePuck_position;
        geometry_msgs::Point greenPole_position;
		

		//declare Node
		ros::NodeHandle node;

		//declare publishers
		ros::Publisher velocity_Publisher;

		//declare subscribers
		ros::Subscriber yellowPuck_subscriber;
		ros::Subscriber bluePuck_subscriber;
        ros::Subscriber greenPole_subscriber;

		//mover forward
		void move(double velocity_speed, double angular_speed);

		//check the current puck position and move accordingly
        void checkPuckPosition(geometry_msgs::Point yellowPuck_position, geometry_msgs::Point bluePuck_position, geometry_msgs::Point greenPole_position);

		//get alls found pucks and save them to puck_1, puck_2 and puck_3		
        void getYellowPuckPositions(const gruppe6::Puck& foundPucks);
        //get alls found pucks and save them to puck_1, puck_2 and puck_3
        void getBluePuckPositions(const gruppe6::Puck& foundPucks);
        //get alls found Poles
        void getGreenPolePositions(const gruppe6::Pole& foundPoles);
};

#endif // PUCKMOVER_H
