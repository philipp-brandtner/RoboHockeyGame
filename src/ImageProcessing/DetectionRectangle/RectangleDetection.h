#ifndef RECTANGLEDETECTION_H
#define RECTANGLEDETECTION_H

//ROS specific includes
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>

//OPENCV specific includes
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

//PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>

//Custom Message includes
#include "gruppe6/Puck.h"

//Custom Service includes
#include "gruppe6/srvRectangleDetection.h"

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

class RectangleDetection
{
public:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    //image_transport::ImageTransport it2_;
    image_transport::Subscriber Image_sub_;
    //image_transport::Subscriber image_sub2_;
    image_transport::Publisher image_pub_;
    // Subscriber for PointCloud
    ros::Subscriber PointCloud_sub;
    // TeamColor Service
    ros::ServiceServer srvRectangleDetection;

    struct HSV_filter_values {
        int iLowH;
        int iHighH;

        int iLowS;
        int iHighS;

        int iLowV;
        int iHighV;
    };

    HSV_filter_values HSV_yellow, HSV_blue;

    Mat originalImg, imgHSV, imgThreshold_Yellow, imgThreshold_Blue, imgCropped;

    RectangleDetection();                                                // Constructor of ImageProcessing Class
    ~RectangleDetection();                                                   // Destructor of ImageProcessing Class
    
private:
    // private Variables
    bool PointCloud_Update;                                         // Handler for Pointcloud Update     
    sensor_msgs::PointCloud2 PointCloud_Input;
    int countBlueRect, countYellowRect, updateBlueRect, updateYellowRect;
    bool updateYellow, updateBlue;
    double areaBlue, areaYellow;
    //extern geometry_msgs::Point pt[];

    // private Methods
    void createControlbar();
    void loadImage(const sensor_msgs::ImageConstPtr& msg);
    void ColorFilter();
    void RectangleApproxBlue();
    void RectangleApproxYellow();
    void CropImage();
    void decideColor();
    bool SrvCall(gruppe6::srvRectangleDetection::Request  &req, gruppe6::srvRectangleDetection::Response &res);
};

#endif // RECTANGLEDETECTION_H
