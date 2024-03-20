#ifndef PUCKDETECTION_H
#define PUCKDETECTION_H

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

//PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>

//Custom Message includes
#include "gruppe6/Puck.h"

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

class PuckDetection
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

    ros::Publisher puck_pub;
    ros::Publisher filteredImage_pub;

    struct HSV_filter_values {
        int iLowH;
        int iHighH;

        int iLowS;
        int iHighS;

        int iLowV;
        int iHighV;
    };

    HSV_filter_values HSV_blue;

    Mat originalImg, imgHSV, imgThreshold_Blue,  imgCropped, RectangleBlue;

    PuckDetection();                                                // Constructor of ImageProcessing Class
    ~PuckDetection();                                                   // Destructor of ImageProcessing Class
    
private:
    // private Variables
    bool PointCloud_Update;                                         // Handler for Pointcloud Update     
    sensor_msgs::PointCloud2 PointCloud_Input;
    //extern geometry_msgs::Point pt[];
    //bool PuckDetectionBlue_update;                                  // Handler for ImageUpdate

    // private Methods
    void createControlbar();
    void loadImage(const sensor_msgs::ImageConstPtr& msg);
    void loadPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg);
    void CropImage();
    void ColorFilter();
    void RectangleApprox();
    void PointCloudProcessing(geometry_msgs::Point pt[], int count);
    void CreatOutMsg(geometry_msgs::Point pt[], int count, float distance[3], pcl::PointCloud<pcl::PointXYZ> cloud);
    void showImages();
};

#endif // PUCKDETECTION_H
