/*
provides services for near field navigation
@author: Simon Wölzmülller
*/

#include "nav_local.h"

/******************************************************************************************
 ************************************* CHECK THE PUCK IN THE FRONT ************************
 ******************************************************************************************/
bool Local_Navigation::checkPuck(bool color)
{
    team_color = color;
    subscribeRGBImage();
    return centered_obj_is_correct();
    unsubscribeRGBImage();
}

bool Local_Navigation::checkPuck(gruppe6::CheckPuck::Request  &req, gruppe6::CheckPuck::Response &res)
{
    team_color = req.teamColor;
    subscribeRGBImage();
    return centered_obj_is_correct();
    unsubscribeRGBImage();
}


/******************************************************************************************
 ************************************* CATCH THE PUCK  ************************************
 ******************************************************************************************/

/**
 * @brief Local_Navigation::collectPuck dummy for testing
 * @return
 */
bool Local_Navigation::catchPuck_start(bool color)
{

    commandPub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);
    catchpuck_status = false;
    team_color = color;
    //if it isn't already in the picturerotate until puck in the picture
    // TODO: look if there's a better direction

    //while(!success){
//        subscribeRGBImage();
//        while(!in_picture()){
//            ROS_INFO("rotating");
//            drive_angular(CLOCKWISE,ROT_ANGLE,PICKUP_VEL_ANG);
//        }
//        drive_angular(CLOCKWISE,29.0,PICKUP_VEL_ANG);
//        TOO_MUCH=0.7;
//        bool direction;
    subscribeLidar(5.0);

//        while(!centered())
//        {
//            direction = (puck_pos==LEFT)?COUNTERCLOCKWISE:CLOCKWISE;
//            drive_angular(direction, ROT_ANGLE_CORR, PICKUP_VEL_ANG_CORR);
//        }
        //unsubscribeRGBImage();
        //printf("in picture\n");

        while(distance_in_center>NEAR_DISTANCE)
        {
          //drive_trans(PICKUP_VEL_FORW,FORWARD);
          if(distance_in_center>NEAR_DISTANCE)catchpuck_status=true;
          ros::spin();
        }

    commandPub.shutdown();

}

/**
 * @brief Local_Navigation::catchPuck_status
 * @param req
 * @param res
 * @return
 */
bool Local_Navigation::catchPuck_status(gruppe6::CatchStatus::Request  &req, gruppe6::CatchStatus::Response &res)
{
   while(!catchpuck_status){
       res.success = false;
   }
   res.success = true;
}

/**
 * @brief Local_Navigation::catchPuck_stop
 * @param req
 * @param res
 * @return
 */
bool Local_Navigation::catchPuck_stop(gruppe6::CatchStop::Request  &req, gruppe6::CatchStop::Response &res)
{
    commandPub.shutdown();
}


/**
 * @brief center a puck in front of the bot and catch it
 *
 */
bool Local_Navigation::catchPuck_start(gruppe6::CatchStart::Request  &req, gruppe6::CatchStart::Response &res)
{
    // TODO: adapt on tested version
    // if it isn't already in the picturerotate until puck in the picture
    while(!in_picture()){
        ROS_INFO("rotating");
        //drive(puck_pos*ROT_ANGLE,NO_FORWARD);

    }
    while(distance_in_center>NEAR_DISTANCE)
    {
        if(!centered()){
            puck_pos = 1.0;
            //drive(puck_pos*ROT_ANGLE_CORR, NO_FORWARD);
            ROS_INFO("not centered");
        }


        //drive(0.0, FORWARD*PICKUP_VEL_FORW);
    }

}


/******************************************************************************************
 ************************************* DROP THE PUCK **************************************
 ******************************************************************************************/
/**
 * @brief drop a puck, if it should be left in the goal position (without rotating)
 *
 */
bool Local_Navigation::dropPuck_start(gruppe6::DropStart::Request  &req, gruppe6::DropStart::Response &res){
    droppuck_status = false;
    commandPub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);
    subscribeLidar(10.0);
    while(distance_in_center<PRE_CATCH_DISTANCE){
        drive_trans(0.1,BACKWARD);
    }
    droppuck_status = true;
    unsubscribeLidar();
    commandPub.shutdown();
}

/**
 * @brief drop a puck, if it should be left in the goal position (without rotating)
 *
 */
bool Local_Navigation::dropPuck_start(){
    distance_in_center=0.0;
    droppuck_status = false;
    commandPub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);
    drive_trans(0.3,BACKWARD);
    drive_angular(0.3,90.0,PICKUP_VEL_ANG);
    droppuck_status = true;
    unsubscribeLidar();
    commandPub.shutdown();

}

/**
 * @brief Local_Navigation::dropPuck_stop
 * @param req
 * @param res
 * @return
 */
bool Local_Navigation::dropPuck_stop(gruppe6::DropStop::Request  &req, gruppe6::DropStop::Response &res)
{
    commandPub.shutdown();
}

/**
 * @brief Local_Navigation::dropPuck_status
 * @param req
 * @param res
 * @return
 */
bool Local_Navigation::dropPuck_status(gruppe6::DropStatus::Request  &req, gruppe6::DropStatus::Response &res)
{
    while(!droppuck_status){
        res.success = false;
    }
    res.success = true;
}


/**
 * @brief collect_puck catches  puck that is in near field before transporting it to the goal
 * @return boolean 1 if success 0 if not
 */
bool Local_Navigation::in_picture()
{
    //printf("%d \n", puck_count);
    if(puck_count)
    {
        return true;
    }
    else{
        return false;
    }
}

/**
 * @brief Local_Navigation::centered checks the mean distance a center range is filled
 * @return
 */
bool Local_Navigation::centered()
{
    return centered_obj_is_correct();
}

/**
 * @brief proof if in center object is the right object
 *
 */
bool Local_Navigation::centered_obj_is_correct()
{
    if(puck_count){
        for (int i=0; i<puck_count;i++){
            if (HALF_PIC_X-PUCK_RADIUS<pt[i].x<HALF_PIC_X+PUCK_RADIUS){return true;ROS_INFO_STREAM("right_puck:"<<pt[i].x);}
        }
    }
    ROS_INFO("wrong_puck");
    return false;
}

/**
 * @brief Local_Navigation::subscribeRGBImage
 */
void Local_Navigation::subscribeRGBImage()
{
    sub2 = n.subscribe("/camera/rgb/image_rect_color", 1, &Local_Navigation::filtImageCallback, this);
}

/**
 * @brief Local_Navigation::unsubscribeRGBImage
 */
void Local_Navigation::unsubscribeRGBImage()
{
    sub2.shutdown();
}


/**
 * @brief Local_Navigation::subscribeLidar subscribe to lidar topic and adapt scan angle
 * @param angle
 */
void Local_Navigation::subscribeLidar(double angle)
{
    min_scan_angle = radians(-(angle/2.0f));
    max_scan_angle = radians(angle/(2.0f));
    sub = n.subscribe("lidarScan", 1, &Local_Navigation::catchLidarCallback, this);
}

/**
 * @brief Local_Navigation::unsubscribeLidar release topic for lidar scanner
 */
void Local_Navigation::unsubscribeLidar()
{
    sub.shutdown();
}


/******************************************************************************************
 ************************************* LIDAR PROCESSING ***********************************
 ******************************************************************************************/

/**
 *  @brief: callback function for lidar sensor data: scans a range and looks for the mean distance in that range
 */
void Local_Navigation::catchLidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    ROS_INFO("START");
    // Find the closest range between the defined minimum and maximum angles
    int minIndex = getMinIndexLidar(scan->angle_min+M_PI,scan->angle_increment, min_scan_angle);
    int maxIndex = getMaxIndexLidar(scan->angle_min+M_PI,scan->angle_increment, max_scan_angle);
    printf("SCAN: ");
    double sum = scan->ranges[minIndex+1];
    printf("%f, ",scan->ranges[minIndex+1]);
    int i=1;

    for (int currIndex = minIndex + 2; currIndex < maxIndex; currIndex++) {
        double x = scan->ranges[currIndex];
        printf("%f, ",scan->ranges[currIndex]);

        if((x>scan->range_min)&&(x<TOO_MUCH)&&(x<scan->range_max)&&isFiniteNumber(x)){
          sum+=scan->ranges[currIndex];
            //printf("%f, ",scan->ranges[currIndex]);
          i++;
        }

    }
    printf(" /  mean: %f \n",sum/(double)i);
    distance_in_center = sum/(double)i;
    //printf("distance in center: %f\n",distance_in_center);
}

/**
 *  @brief: give this function the horizontal distance of a element in a image from
 *  the right border and you get the phi angle from lidar left side
 */
double Local_Navigation::x2phi(double x)
{
    double phi_u = acos(HALF_PIC_X - x);
    phi_u = 180.0 - phi_u;
    return abs(PHI_EDGE_CAM - phi_u);
}

/**
 *  @brief: give this function the angle with respect to left image side, get the
 * index of the lidar scan to this angle
 */
double Local_Navigation::phi2index(double phi){
    // Maybe we don't need that
    return 0;
}


/**
 *  @brief: returns the minimum index, for the range, used in lidarCallback
 */
int Local_Navigation::getMinIndexLidar(double min_angle, double increment, double min_scan_angle_rad){

    int minIndex = ceil((min_scan_angle_rad - min_angle) / increment);
    lidar_min_index = minIndex;
    lidar_increment = increment;

    return minIndex;
}

/**
 *  @brief: returns the maximum index, for the range, used in lidarCallback
 */
int Local_Navigation::getMaxIndexLidar(double min_angle, double increment, double max_scan_angle_rad){

    int maxIndex = floor((max_scan_angle_rad - min_angle) / increment);
    lidar_max_index = maxIndex;
    lidar_increment = increment;

    return maxIndex;
}

bool Local_Navigation::isFiniteNumber(float x)
{
     return (x <= DBL_MAX);
}


/******************************************************************************************
 ************************************* IMAGE PROCESSING ***********************************
 ******************************************************************************************/
/**
 *  @brief: Callback function for filtered Image.
 */
void Local_Navigation::filtImageCallback(const sensor_msgs::ImageConstPtr& msg)
{	
    //ROS_INFO("STARTING");
    for(int i = 0; i<=2;i++){
        pt[i] = EMPTY_POINT;
    }
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        //ROS_INFO("GOT THE IMAGE");
    }
    catch (cv_bridge::Exception& e)
    {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
    }
    originalImg = cv_ptr->image;
    colorFilter();
    Rect croppedRectangle = Rect(0,originalImg.rows/2,originalImg.cols,originalImg.rows/2);
    imgCropped = imgThreshold(croppedRectangle);
    
    
    double ratio, area;
    int thresh = 100;
    Mat contoursInput=imgCropped;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    /// Find contours
    findContours( contoursInput, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Approximate contours to polygons + get bounding rects
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    int count = contours.size();
    puck_count = 0;
    /// Find the rotated rectangles and ellipses for each contour
    vector<RotatedRect> minRect( contours.size() );
    int k = 0;
    int index_of_middlest = 0;
    int min_dist_to_middle=641;
    for( int i = 0; i < count; i++ ) {
        minRect[i] = minAreaRect( Mat(contours[k]) );
        if(minRect[i].angle < -45.0) {
                ratio = (double)((double)minRect[i].size.width/(double)minRect[i].size.height);
        }
        else {
                ratio = (double)((double)minRect[i].size.height/(double)minRect[i].size.width);
        }
        area = (double)((double)minRect[i].size.height*(double)minRect[i].size.width);

        if(area < 500.0 || ratio > 3.1 || ratio < 0.9) {
                i--;
                count--;
        }
        else {
                pt[i].x = minRect[i].center.x;
                pt[i].y = minRect[i].center.y + 240;
                pt[i].z=count;
                if (abs(HALF_PIC_X-minRect[i].center.x)<min_dist_to_middle){
                    index_of_middlest = i;
                    min_dist_to_middle = abs(HALF_PIC_X-minRect[i].center.x);
                }
        }
        k++;
    }

    //define the position of the nearest puck!
    if(count>0){
        if(pt[index_of_middlest].x<HALF_PIC_X-CENTER_DEVIATION){puck_pos = LEFT;ROS_INFO("LEFT");}
        if(HALF_PIC_X-CENTER_DEVIATION<pt[index_of_middlest].x<HALF_PIC_X+CENTER_DEVIATION){puck_pos = CENTER;ROS_INFO("CENTER");}
        if(pt[index_of_middlest].x>HALF_PIC_X+CENTER_DEVIATION){puck_pos = RIGHT;ROS_INFO("RIGHT");}
        ROS_INFO_STREAM("Found Puck @ "<<pt[index_of_middlest].x);
    }

    // how many pucks are in the camera field
    puck_count = count;
    //printf("count of objects: %d\n", count);
}

/**
 * @brief Local_Navigation::colorFilter filters the rgb image
 */
void Local_Navigation::colorFilter()
{
    cvtColor(originalImg, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    HSV_filter_values HSV_color = team_color==BLUE?HSV_blue:HSV_yellow;
    //printf("color = %d\n",team_color);
    //Threshold the images with the color
    inRange(imgHSV, Scalar(HSV_color.iLowH, HSV_color.iLowS, HSV_color.iLowV), Scalar(HSV_color.iHighH, HSV_color.iHighS, HSV_color.iHighV), imgThreshold);
    int factor = 10;
    //morphological opening (remove small objects from the foreground)
    erode(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(factor, factor)) );
    dilate( imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(factor, factor)) );

    //morphological closing (fill small holes in the foreground)
    dilate( imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(factor,factor)) );
    erode(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(factor, factor)) );

    //imshow("contours",imgThreshold);
    //waitKey(1);
}

/******************************************************************************************
 ************************************* DRIVING ROUTINES ***********************************
 ******************************************************************************************/


/**
 * @brief Local_Navigation::drive_angular
 * @param angle in degrees
 */
void Local_Navigation::drive_angular(bool clockwise,double angle, double vel){
    //msg of type to say the bot where to twist
    geometry_msgs::Twist act_msg;
    // angle in z direction
    double rot_angle = angle;
    // velocity in rotation
    double rot_vel = radians(vel);
    // set translateral velocity TO zero
    act_msg.linear.x = 0.0;
    act_msg.linear.y = 0.0;
    act_msg.linear.z = 0.0;
    // let's adjust angular coordinates
    act_msg.angular.x = 0.0;
    act_msg.angular.y = 0.0;
    // distinguish rotation direction
    if (rot_angle>0){act_msg.angular.z = fabs(rot_vel);}

    else {act_msg.angular.z = -fabs(rot_vel);}
    //printf("Rotangle: %f \n", rot_angle);
    // we need the current time for distance calculation
    double t0 = ros::Time::now().toSec();
    double t1 = 0.0;
    double temp_angle = 0.0;
    // how often will we send the message out?
    ros::Rate loop_rate(ROT_LOOP_RATE);
    do{
        // publish action message
        commandPub.publish(act_msg);
        // stop time
        t1=ros::Time::now().toSec();
        // use it to compute passed distance
        temp_angle = abs(degrees(rot_vel))*(t1-t0);
        // apply spin
        ros::spinOnce();
        // wait (see loop_rate)
        loop_rate.sleep();

    }while(abs(temp_angle) < abs(angle)); // until the wanted distance is passed
    // from now on send a stop message
    act_msg.angular.z = 0;
    // publish action message
    commandPub.publish(act_msg);

}

/**
 * @brief Local_Navigation::drive driving routine
 * @param angle rotation angle
 * @param forward (-1, 0, 1) for backward, stay, forward
 */
void Local_Navigation::drive_trans(double distance, double forward)
{
    ROS_INFO("Driving");
    //msg of type to say the bot where to twist
    geometry_msgs::Twist act_msg;
    // set translateral velocity
    act_msg.linear.x = PICKUP_VEL_FORW*forward;
    act_msg.linear.y = 0.0;
    act_msg.linear.z = 0.0;
    double t0 = ros::Time::now().toSec();
    double t1 = 0.0;
    double temp_dist = 0.0;
    ros::Rate loop_rate(ROT_LOOP_RATE);
    while(temp_dist<distance){
        commandPub.publish(act_msg);
        t1=ros::Time::now().toSec();
        // use it to compute passed distance
        temp_dist = PICKUP_VEL_FORW*(t1-t0);
        // apply spin
        ros::spinOnce();
        // wait (see loop_rate)
        loop_rate.sleep();
    }

    act_msg.linear.x = 0;
    // publish action message
    commandPub.publish(act_msg);
}

/**
 * @brief Local_Navigation::~Local_Navigation Destructor
 */
Local_Navigation::~Local_Navigation()
{	
	ROS_INFO("DESTRUCTION");
}
	
/**
 * @brief Local_Navigation::Local_Navigation Constructor
 */
Local_Navigation::Local_Navigation()
{
    // Initializing of HSV values for each color
    HSV_yellow.iLowH = 16;
    HSV_yellow.iHighH = 38;
    HSV_yellow.iLowS = 41;
    HSV_yellow.iHighS = 255;
    HSV_yellow.iLowV = 171;
    HSV_yellow.iHighV = 255;

    HSV_blue.iLowH = 108;
    HSV_blue.iHighH = 131;
    HSV_blue.iLowS = 126;
    HSV_blue.iHighS = 255;
    HSV_blue.iLowV = 39;
    HSV_blue.iHighV = 124;

    ROS_INFO("NL-Puck-Aufnehmer wird vorbereitet");
    EMPTY_POINT.x = 0.0;
    EMPTY_POINT.y = 0.0;
    EMPTY_POINT.z = 0.0;
    puck_count = 0;
    puck_count_temp = 0;
    distance_in_center = 100;
    puck_pos = LEFT;

    // create callable services for master
    collect_start = n.advertiseService("catch_puck_start", &Local_Navigation::catchPuck_start, this);
    drop_start = n.advertiseService("drop_puck_start", &Local_Navigation::dropPuck_start,this);
    check = n.advertiseService("check_puck", &Local_Navigation::checkPuck,this);
    collect_stop = n.advertiseService("catch_puck_stop", &Local_Navigation::catchPuck_stop, this);
    drop_stop = n.advertiseService("drop_puck_stop", &Local_Navigation::dropPuck_stop,this);
    collect_status = n.advertiseService("catch_puck_status", &Local_Navigation::catchPuck_status, this);
    drop_status = n.advertiseService("drop_puck_status", &Local_Navigation::dropPuck_status,this);
    catchPuck_start(BLUE);
    //checkPuck(BLUE);
    //dropPuck_start();
    ROS_INFO("NL-Puck-Aufnehmer fertig!");
}

