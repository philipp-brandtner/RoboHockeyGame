#include "puck_mover.h"

PuckMover::PuckMover()
{
	EMPTY_POINT.x = 0.0;
    EMPTY_POINT.y = 0.0;
    EMPTY_POINT.z = 0.0;
    poleDistance = 0.8;
	positionReached = false;
	puckFound = false;
	startStop_PuckMover = false;
	//Subscribe to puck topics
    yellowPuck_subscriber = node.subscribe("/puck_detection/yellow", 1, &PuckMover::getYellowPuckPositions, this);
    bluePuck_subscriber = node.subscribe("/puck_detection/blue", 1, &PuckMover::getBluePuckPositions, this);
    greenPole_subscriber = node.subscribe("/pole_detection/green", 1, &PuckMover::getGreenPolePositions, this);
	//register the velocity publisher
	velocity_Publisher =node.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);
}

void PuckMover::startMoving()
{	
	ROS_INFO("START PUCK MOVER");
	ros::Rate rate(10);
	while(ros::ok())
	{	
        checkPuckPosition(yellowPuck_position, bluePuck_position, greenPole_position);
		rate.sleep();	
	}
	ROS_INFO("END PUCK MOVER");
}


void PuckMover::getYellowPuckPositions(const gruppe6::Puck &foundPucks)
{
    //ROS_INFO_STREAM("Get Puck 1 Z:   " << foundPucks.puck_1.z);
	//ROS_INFO_STREAM("Get Puck 2 Z:   " << foundPucks.puck_2.z);
	//ROS_INFO_STREAM("COUNT   " << foundPucks.count);
	
    float_t puck1 = foundPucks.puck_1.z;
    float_t puck2 = foundPucks.puck_2.z;
    float_t puck3 = foundPucks.puck_3.z;

	if(!positionReached)
	{
		if(foundPucks.count > 0)
		{
			puckFound = true;
		}
		
		switch(foundPucks.count)
		{
			case 1:	
				yellowPuck_position = foundPucks.puck_1;
				//ROS_INFO("PUCK_1");
				break;
            case 2:
                if(puck1 < puck2)
				{
				//	ROS_INFO("PUCK_1");
                    yellowPuck_position = foundPucks.puck_1;
				}
				else
				{
				//	ROS_INFO("PUCK_2");
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
				//ROS_INFO("NO PUCK ==> Turn");
				yellowPuck_position = EMPTY_POINT;
				break;
		}
	}	
}

void PuckMover::getBluePuckPositions(const gruppe6::Puck &foundPucks)
{
	float_t puck1 = foundPucks.puck_1.z;
	float_t puck2 = foundPucks.puck_2.z;
	float_t puck3 = foundPucks.puck_3.z;
	
	//ROS_INFO_STREAM("BluePuck1: " << puck1<< "\nBluePuck2: " << puck2 << "\nBluePuck3: " << puck3);
	
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
			//ROS_INFO("NO PUCK ==> Turn");
			bluePuck_position = EMPTY_POINT;
			break;
	}
}

void PuckMover::getGreenPolePositions(const gruppe6::Pole &foundPoles)
{
    float_t pole1 = foundPoles.pole_1.z;
    float_t pole2 = foundPoles.pole_2.z;
    float_t pole3 = foundPoles.pole_3.z;
    float_t pole4 = foundPoles.pole_4.z;
    float_t pole5 = foundPoles.pole_5.z;
    float_t pole6 = foundPoles.pole_6.z;
    float_t pole7 = foundPoles.pole_7.z;
    float_t pole8 = foundPoles.pole_8.z;

    switch(foundPoles.count)
    {
        case 1:
            greenPole_position = foundPoles.pole_1;
            break;

        case 2:
            if(pole1 < pole2)
            {
                greenPole_position = foundPoles.pole_1;
            }
            else
            {
                greenPole_position = foundPoles.pole_2;
            }
            break;

        case 3:
            if(pole1 < pole2 && pole1 < pole3 )
            {
                greenPole_position = foundPoles.pole_1;
            }
            else if(pole2 < pole1 && pole2 < pole3)
            {
                greenPole_position = foundPoles.pole_2;
            }
            else
            {
                greenPole_position = foundPoles.pole_3;
            }
            break;

        case 4:
            if(pole1 < pole2 && pole1 < pole3 && pole1 < pole4)
            {
                greenPole_position = foundPoles.pole_1;
            }
            else if(pole2 < pole1 && pole2 < pole3 && pole2 < pole4)
            {
                greenPole_position = foundPoles.pole_2;
            }
            else if (pole3 < pole1 && pole3 < pole2 && pole3 < pole4)
            {
                greenPole_position = foundPoles.pole_3;
            }
            else
            {
                greenPole_position = foundPoles.pole_4;
            }
            break;

        case 5:
            if(pole1 < pole2 && pole1 < pole3 && pole1 < pole4 && pole1 < pole5)
            {
                greenPole_position = foundPoles.pole_1;
            }
            else if(pole2 < pole1 && pole2 < pole3 && pole2 < pole4 && pole2 < pole5)
            {
                greenPole_position = foundPoles.pole_2;
            }
            else if (pole3 < pole1 && pole3 < pole2 && pole3 < pole4 && pole3 < pole5)
            {
                greenPole_position = foundPoles.pole_3;
            }
            else if (pole4 < pole1 && pole4 < pole2 && pole4 < pole3 && pole4 < pole5)
            {
                greenPole_position = foundPoles.pole_4;
            }
            else
            {
                greenPole_position = foundPoles.pole_5;
            }
            break;

        case 6:
            if(pole1 < pole2 && pole1 < pole3 && pole1 < pole4 && pole1 < pole5 && pole1 < pole6)
            {
                greenPole_position = foundPoles.pole_1;
            }
            else if(pole2 < pole1 && pole2 < pole3 && pole2 < pole4 && pole2 < pole5 && pole2 < pole6)
            {
                greenPole_position = foundPoles.pole_2;
            }
            else if (pole3 < pole1 && pole3 < pole2 && pole3 < pole4 && pole3 < pole5 && pole3 < pole6)
            {
                greenPole_position = foundPoles.pole_3;
            }
            else if (pole4 < pole1 && pole4 < pole2 && pole4 < pole3 && pole4 < pole5 && pole4 < pole6)
            {
                greenPole_position = foundPoles.pole_4;
            }
            else if (pole5 < pole1 && pole5 < pole2 && pole5 < pole3 && pole5 < pole4 && pole5 < pole6)
            {
                greenPole_position = foundPoles.pole_5;
            }
            else
            {
                greenPole_position = foundPoles.pole_6;
            }
            break;

        case 7:
            if(pole1 < pole2 && pole1 < pole3 && pole1 < pole4 && pole1 < pole5 && pole1 < pole6 && pole1 < pole7)
            {
                greenPole_position = foundPoles.pole_1;
            }
            else if(pole2 < pole1 && pole2 < pole3 && pole2 < pole4 && pole2 < pole5 && pole2 < pole6 && pole2 < pole7)
            {
                greenPole_position = foundPoles.pole_2;
            }
            else if (pole3 < pole1 && pole3 < pole2 && pole3 < pole4 && pole3 < pole5 && pole3 < pole6 && pole3 < pole7)
            {
                greenPole_position = foundPoles.pole_3;
            }
            else if (pole4 < pole1 && pole4 < pole2 && pole4 < pole3 && pole4 < pole5 && pole4 < pole6 && pole4 < pole7)
            {
                greenPole_position = foundPoles.pole_4;
            }
            else if (pole5 < pole1 && pole5 < pole2 && pole5 < pole3 && pole5 < pole4 && pole5 < pole6 && pole5 < pole7)
            {
                greenPole_position = foundPoles.pole_5;
            }
            else if (pole6 < pole1 && pole6 < pole2 && pole6 < pole3 && pole6 < pole4 && pole6 < pole5 && pole6 < pole7)
            {
                greenPole_position = foundPoles.pole_6;
            }
            else
            {
                greenPole_position = foundPoles.pole_7;
            }
            break;

        case 8:
            if(pole1 < pole2 && pole1 < pole3 && pole1 < pole4 && pole1 < pole5 && pole1 < pole6 && pole1 < pole7 && pole1 < pole8)
            {
                greenPole_position = foundPoles.pole_1;
            }
            else if(pole2 < pole1 && pole2 < pole3 && pole2 < pole4 && pole2 < pole5 && pole2 < pole6 && pole2 < pole7 && pole2 < pole8)
            {
                greenPole_position = foundPoles.pole_2;
            }
            else if (pole3 < pole1 && pole3 < pole2 && pole3 < pole4 && pole3 < pole5 && pole3 < pole6 && pole3 < pole7 && pole3 < pole8)
            {
                greenPole_position = foundPoles.pole_3;
            }
            else if (pole4 < pole1 && pole4 < pole2 && pole4 < pole3 && pole4 < pole5 && pole4 < pole6 && pole4 < pole7 && pole4 < pole8)
            {
                greenPole_position = foundPoles.pole_4;
            }
            else if (pole5 < pole1 && pole5 < pole2 && pole5 < pole3 && pole5 < pole4 && pole5 < pole6 && pole5 < pole7 && pole5 < pole8)
            {
                greenPole_position = foundPoles.pole_5;
            }
            else if (pole6 < pole1 && pole6 < pole2 && pole6 < pole3 && pole6 < pole4 && pole6 < pole5 && pole6 < pole7 && pole6 < pole8)
            {
                greenPole_position = foundPoles.pole_6;
            }
            else if (pole7 < pole1 && pole7 < pole2 && pole7 < pole3 && pole7 < pole4 && pole7 < pole5 && pole7 < pole6 && pole7 < pole8)
            {
                greenPole_position = foundPoles.pole_7;
            }
            else
            {
                greenPole_position = foundPoles.pole_8;
            }
            break;

        default:
            //ROS_INFO("NO PUCK ==> Turn");
            greenPole_position = EMPTY_POINT;
            break;
    }
}


void PuckMover::move(double velocity_speed, double angular_speed)
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


void PuckMover::checkPuckPosition(geometry_msgs::Point yellowPuck_position, geometry_msgs::Point bluePuck_position, geometry_msgs::Point greenPole_position)
{	
    //ROS_INFO_STREAM("puck at Z:   " << puck_position.z);
	//ROS_INFO_STREAM("puck at Y:   " << puck_position.y);
	//ROS_INFO_STREAM("puck at X:   " << puck_position.x);

	double vel_speed = 0.0;
	double angular_speed = 0.0;
	
    if(!puckFound)
	{
		ROS_INFO("NO PUCK Turn Right");
		angular_speed = -0.4;
	}
	else
	{
        ROS_INFO_STREAM("Green Position Z: " << greenPole_position.z << "Position X: " << greenPole_position.x);
        ROS_INFO_STREAM("Blue Position Z: " << bluePuck_position.z << "Position X: " << bluePuck_position.x);
        ROS_INFO_STREAM("Yellow Position Z: " << yellowPuck_position.z << "Position X: " << yellowPuck_position.x);
        //check if the way between robot and searched puck is not blocked
        if((bluePuck_position.z == 0 || yellowPuck_position.z < bluePuck_position.z) &&
                (greenPole_position.z == 0 || yellowPuck_position.z < greenPole_position.z || greenPole_position.x > (yellowPuck_position.x + poleDistance) || (greenPole_position.x < (yellowPuck_position.x - poleDistance))))
		{		
            ROS_INFO("NO BLOCKING OBJECTS DETECTED");
            if(yellowPuck_position.z >= 0.5 && !positionReached)
			{
				vel_speed = 0.1;
								
				if(yellowPuck_position.x > 0.03)
				{
                    //ROS_INFO("Drive forward RIGHT");
					angular_speed = -0.15;
				}
				else if(yellowPuck_position.x < -0.03)
				{
                    //ROS_INFO("Drive forward LEFT");
					angular_speed = 0.15;
				}
				else
				{
                    //ROS_INFO("Drive forward1");
					angular_speed = 0.0;
                }

                //save last yellow puck position to check if the robot stops not a near field of puck
                 last_yellowPuck_position = yellowPuck_position;
			}
			else
            {
                if(last_yellowPuck_position.z > 0.6)
                {
                    ROS_INFO("LOST PUCK");
                }
                else
                {
                    ROS_INFO("Position Reached");
                    positionReached = true;
                }
			}			
		}
        else //The way to the searched puck is blocked (blue puck or/and green pole in between)
		{
            if(bluePuck_position.z > 0 && bluePuck_position.x < yellowPuck_position.x && bluePuck_position.x > (yellowPuck_position.x - 0.15))
			{
                ROS_INFO("TRY =======> Turn RIGHT");
                if(greenPole_position.z == 0 || (greenPole_position.z > yellowPuck_position.z || ((greenPole_position.x < yellowPuck_position.x) || (greenPole_position.x > (yellowPuck_position.x + poleDistance)))))
                {
                    vel_speed = 0.1;
                    ROS_INFO("NO Pole ===> Turn RIGHT");
                    angular_speed = -0.15;
                }
                else
                {
                    angular_speed = 0.15;
                    ROS_INFO("Problem ====>>> stop the robot and turn left till find a new puck");
                }
			}

            ROS_INFO("====================================");

            //save last yellow puck position to check if the robot stops not a near field of puck
            last_yellowPuck_position = yellowPuck_position;


       /*     else if(bluePuck_position.x > (yellowPuck_position.x + 0.05))
			{
                if(greenPole_position.z != 0 && greenPole_position.z > yellowPuck_position.z)
                {
                    vel_speed = 0.1;
                    ROS_INFO("Turn LEFT");
                    angular_speed = 0.15;
                }
                else
                {
                    //&& greenPole_position.x < (yellowPuck_position.x - poleDistance)
                    ROS_INFO("Problem ====>>> stop the robot and turn right till find a new puck");
                }
            }




			else
			{
                if(yellowPuck_position.z >= 0.4 && !positionReached)
				{
					vel_speed = 0.1;
									
					if(yellowPuck_position.x > 0.03)
					{
                        //ROS_INFO("Drive forward RIGHT");
						angular_speed = -0.15;
					}
					else if(yellowPuck_position.x < -0.03)
					{
                        //ROS_INFO("Drive forward LEFT");
						angular_speed = 0.15;
					}
					else
					{
                        //ROS_INFO("Drive forward2");
						angular_speed = 0.0;
					}			
				}
				else
                {

                    ROS_INFO("Position Reached");
					positionReached = true;
				}	
			}
            */
        }
    }

	
	//ROS_INFO_STREAM("vel speed:" << vel_speed);
	//ROS_INFO_STREAM("angl speed: " << angular_speed);
	move(vel_speed, angular_speed);	
}








