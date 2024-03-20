#ifndef NAV_LOCAL_HPP
#define NAV_LOCAL_HPP

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "gruppe6/Puck.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// SERVICE HEADERS
#include "gruppe6/CatchStart.h"
#include "gruppe6/CatchStop.h"
#include "gruppe6/CatchStatus.h"
#include "gruppe6/DropStart.h"
#include "gruppe6/DropStop.h"
#include "gruppe6/DropStatus.h"
#include "gruppe6/CheckPuck.h"
#include <vector>
#include <algorithm>
#include <math.h>
#include <cmath>

using namespace cv;
using namespace std;

#define FORWARD             (double)0.5
#define NO_FORWARD          (double)0.0
#define BACKWARD            (double)-0.5
#define ZERO                (float)0.0
#define LEFT                (double)1.0
#define CENTER              (double)0.0
#define RIGHT               (double)1.0
#define YELLOW              1
#define BLUE                0
#define CLOCKWISE           1
#define COUNTERCLOCKWISE    0
#define radians(deg)        deg * M_PI / 180
#define degrees(rad)        rad * 180 / M_PI




class Local_Navigation {
    public:
        // Constructors
        Local_Navigation(double speed, double angle, double rot_vel);
        Local_Navigation(double speed, double angle);
        Local_Navigation(double speed);
        Local_Navigation();
        ~Local_Navigation();
        struct HSV_filter_values {
               int iLowH;
               int iHighH;
               int iLowS;
               int iHighS;
               int iLowV;
               int iHighV;
       };
        // node and subscribing
        ros::Subscriber sub;
        ros::Subscriber sub2;
        ros::Publisher commandPub;
        ros::NodeHandle n;

    private:

        // Services
        ros::ServiceServer collect_start;
        ros::ServiceServer drop_start;
        ros::ServiceServer check;
        ros::ServiceServer drop_stop;
        ros::ServiceServer collect_stop;
        ros::ServiceServer drop_status;
        ros::ServiceServer collect_status;



        // constants
        const static double PICKUP_VEL_FORW = 0.4;
        const static double PICKUP_VEL_ANG = 30.0;
        const static double PICKUP_VEL_ANG_CORR = 1.0;
        const static double ROT_ANGLE = 29.0;
        const static double ROT_ANGLE_CORR = 5.0;
        const static double ROT_LOOP_RATE = 70;
        const static float NEAR_DISTANCE = 0.24;
        const static double CENTER_DEVIATION = 10;
        const static int THRESHOLD_PUCK_SEEN = 5;
        const static double PRE_CATCH_DISTANCE  = 0.6;

        const static double PHI_CAM = 58.0;
        const static double PHI_EDGE_CAM = 61.0;
        const static double PUCK_DIAMETER = 4.5;
        const static int HALF_PIC_X = 320;
        const static int PUCK_RADIUS = 75;

			
        // dummy point
        geometry_msgs::Point EMPTY_POINT;
        bool catchPuck_start(bool color);
        bool checkPuck(bool color);
        bool dropPuck_start();

        //service functions & status variables
        bool catchPuck_start(gruppe6::CatchStart::Request  &req, gruppe6::CatchStart::Response &res);
        bool catchpuck_status;
        bool catchPuck_stop(gruppe6::CatchStop::Request  &req, gruppe6::CatchStop::Response &res);
        bool catchPuck_status(gruppe6::CatchStatus::Request  &req, gruppe6::CatchStatus::Response &res);
        bool dropPuck_start(gruppe6::DropStart::Request  &req, gruppe6::DropStart::Response &res);
        bool dropPuck_stop(gruppe6::DropStop::Request  &req, gruppe6::DropStop::Response &res);
        bool droppuck_status;
        bool dropPuck_status(gruppe6::DropStatus::Request  &req, gruppe6::DropStatus::Response &res);
        bool checkPuck(gruppe6::CheckPuck::Request  &req, gruppe6::CheckPuck::Response &res);

        // general functions, needed by all services
        void filtImageCallback(const sensor_msgs::ImageConstPtr& msg);
        void catchLidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void drive_trans(double distance, double forward);
        void drive_angular(bool clockwise, double angle, double vel);
        double x2phi(double x);
        double phi2index(double phi);
        int getMinIndexLidar(double min_angle, double increment, double min_scan_angle_rad);
        int getMaxIndexLidar(double min_angle, double increment, double max_scan_angle_rad);
        bool centered();
        bool in_picture();
        bool centered_obj_is_correct();
        void colorFilter();
        void subscribeRGBImage();
        void unsubscribeRGBImage();
        void subscribeLidar(double angle);
        void unsubscribeLidar();
        double angle2rad(double angle);


        bool isFiniteNumber(float x);
        // Data needed by the general functions
        Mat originalImg,imgCropped, imgThreshold, imgHSV;
        geometry_msgs::Point pt[3];
        uint8_t lidar_min_index;
        uint8_t lidar_max_index;
        double lidar_increment;
        float TOO_MUCH;
        uint8_t puck_count;
        uint8_t puck_count_temp;
        double min_scan_angle;
        double max_scan_angle;
        double distance_in_center;
        int puck_pos;
        bool team_color;
        HSV_filter_values HSV_yellow, HSV_blue;
};





#endif // NAV_LOCAL_HPP

