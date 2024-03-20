#include "PoleDetection.h"

/*
 * Constructor for PuckDetection Class
 */
PoleDetection::PoleDetection() : it_(nh_)
{
    ROS_INFO("POLEDETECTION GREEN");
    ROS_INFO("--> Running!");
    ROS_INFO("Echo topic for data.");
    PointCloud_Update = false;
    // Initializing of HSV values for each color

    HSV_green.iLowH = 30;
    HSV_green.iHighH = 83;
    HSV_green.iLowS = 40;
    HSV_green.iHighS = 187;
    HSV_green.iLowV = 107;
    HSV_green.iHighV = 232;

    //createControlbar();

    // Subscrive to input video feed
    Image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1, &PoleDetection::loadImage, this);
    PointCloud_sub = nh_.subscribe("/camera/depth/points", 1, &PoleDetection::loadPointCloud, this);
    // Publish the detected poles
    pole_pub = nh_.advertise<gruppe6::Pole>("/pole_detection/green", 1000);
}

/*
 * Destructor for PoleDetection Class
 */

PoleDetection::~PoleDetection()
{
    ROS_INFO("DESTROY");
    destroyWindow(OPENCV_WINDOW);
}

/*
 * Create Treckbars for HSV value adjustment
 */

void PoleDetection::createControlbar()
{
    namedWindow("Control Green", CV_WINDOW_AUTOSIZE);
    createTrackbar("LowH", "Control Green", &HSV_green.iLowH, 179); //Hue (0 - 179)
    createTrackbar("HighH", "Control Green", &HSV_green.iHighH, 179);

    createTrackbar("LowS", "Control Green", &HSV_green.iLowS, 255); //Saturation (0 - 255)
    createTrackbar("HighS", "Control Green", &HSV_green.iHighS, 255);

    createTrackbar("LowV", "Control Green", &HSV_green.iLowV, 255); //Value (0 - 255)
    createTrackbar("HighV", "Control Green", &HSV_green.iHighV, 255);
}

/*
 * Load Images from RBG Topic
 */

void PoleDetection::loadImage(const sensor_msgs::ImageConstPtr& msg) {
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
    blur(originalImg, originalImg, Size( 4, 4 ) );

    // Filter each color of the Image
    CropImage();
}

/*
 * Load Pointcloud from Pointcloud Topic
 */

void PoleDetection::loadPointCloud(const sensor_msgs::PointCloud2ConstPtr& input) {
    PointCloud_Update=true;

    PointCloud_Input=*input;
}

void PoleDetection::CropImage() {
    //Rect croppedRectangle = Rect(0,originalImg.rows/3,originalImg.cols,originalImg.rows*2/3);
    imgCropped = originalImg;
    ColorFilter();
}

/*
 * Image processing for RBG picture
 * - Convert RGB picture to HSV and apply Filter
 * - Do morphological opening and closing to remove Noise
 */

void PoleDetection::ColorFilter()
{
    //Convert the captured frame from BGR to HSV
    cvtColor(imgCropped, imgHSV, COLOR_BGR2HSV);

    //Threshold the images with the color
    inRange(imgHSV, Scalar(HSV_green.iLowH, HSV_green.iLowS, HSV_green.iLowV), Scalar(HSV_green.iHighH, HSV_green.iHighS, HSV_green.iHighV), imgThreshold_Green);

    //morphological opening (remove small objects from the foreground)
    erode(imgThreshold_Green, imgThreshold_Green, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThreshold_Green, imgThreshold_Green, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    //morphological closing (fill small holes in the foreground)
    dilate( imgThreshold_Green, imgThreshold_Green, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(imgThreshold_Green, imgThreshold_Green, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    RectangleApprox();
}


//durchmesser=6.0
//länge=52.5
//Verhältnis=8.75

/*
 * Rectangle approximation for the individual poles
 */
void PoleDetection::RectangleApprox()
{
    double ratio, area;
    int thresh = 100;
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Find contours
    findContours( imgThreshold_Green, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Approximate contours to polygons + get bounding rects
    int count = contours.size();
    geometry_msgs::Point pt[14];

    /// Find the rotated rectangles and ellipses for each contour
    vector<RotatedRect> minRect( contours.size() );

    int k = 0;
    for( int i = 0; i < count; i++ ) {
        minRect[i] = minAreaRect( Mat(contours[k]) );
        //ROS_INFO("Height: %f",minRect[i].size.height);
        //ROS_INFO("Width: %f",minRect[i].size.width);
        //ROS_INFO("Angle: %f", minRect[i].angle);
        //Check if width and hight have to be exchanged
        if(minRect[i].angle < -45.0) {
            ratio = (double)((double)minRect[i].size.width/(double)minRect[i].size.height);
        }
        else {
            ratio = (double)((double)minRect[i].size.height/(double)minRect[i].size.width);
        }
        area = (double)((double)minRect[i].size.height*(double)minRect[i].size.width);

        //ROS_INFO("Ratio: %f",ratio);
        //ROS_INFO("Area: %f", area);
        ///ROS_INFO("CenterX: %f",minRect[i].center.x);
        //ROS_INFO("CenterY: %f",minRect[i].center.y);
        // First filtering to get rid of small contour areas and areas, which don't fullfill the ratio ... ratio 8.75
        if(area < 50.0 || ratio > 13.0 || ratio < 6.0) { //|| (double)minRect[i].width > 450.0 || (double)minRect[i].height < 70.0) { //ratio < 7.0
            //ROS_INFO("Ratio: %f",ratio);
            //ROS_INFO("Area: %f", area);
            i--;
            count--;
        }
        else {
            //ROS_INFO("Height: %f",minRect[i].size.height);
            //ROS_INFO("Width: %f",minRect[i].size.width);
            pt[i].x = minRect[i].center.x;
            pt[i].y = minRect[i].center.y;
            pt[i].z=count;
        }
        k++;
    }

    //ROS_INFO("Count: %i", count);
    /// Draw contours + rotated rects + ellipses
    RectangleGreen = Mat::zeros( imgThreshold_Green.size(), CV_8UC3 );
    for( int i = 0; i < count; i++ ) {
        Scalar color = Scalar( 0, 255, 140 );
        // rotated rectangle
        Point2f rect_points[4]; minRect[i].points( rect_points );
        for( int j = 0; j < 4; j++ )
          line( RectangleGreen, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
    }

    Point pt_test;
    pt_test.x=640;
    pt_test.y=240;

    Scalar color = Scalar( 0, 140, 140 );
    circle(RectangleGreen, pt_test,5,color,2, 8, 0);

    //showImages();
    if(PointCloud_Update)
        PointCloudProcessing(pt, count);
}

void PoleDetection::PointCloudProcessing(geometry_msgs::Point pt[], int count) {
    float distance[3] = {0.0, 0.0, 0.0};
    gruppe6::Pole out_msg;
    // Container for original & filtered data
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(PointCloud_Input, cloud);
    if (count>=0 && count <= 7 )
        CreatOutMsg(pt,count,distance, cloud);
    else 
        ROS_INFO("ERROR");

    PointCloud_Update=false;
}

void PoleDetection::CreatOutMsg(geometry_msgs::Point pt[], int count, float distance[3], pcl::PointCloud<pcl::PointXYZ> cloud) {
    gruppe6::Pole out_msg;

    /*
     * Start building the output message
     */
    out_msg.header.stamp = PointCloud_Input.header.stamp;
    out_msg.header.frame_id = PointCloud_Input.header.frame_id;
    switch(count) {
        case 0:
            out_msg.count = count;
            pole_pub.publish(out_msg);
            break;
        case 1:
            // Pole 1 Position
            if(cloud.at(pt[0].x, pt[0].y).x == cloud.at(pt[0].x, pt[0].y).x) {          // Check for nan
                out_msg.count = count;
                out_msg.pole_1.x = cloud.at(pt[0].x, pt[0].y).x;
                out_msg.pole_1.y = cloud.at(pt[0].x, pt[0].y).y;
                out_msg.pole_1.z = cloud.at(pt[0].x, pt[0].y).z;

                pole_pub.publish(out_msg);
            }
            break;
        case 2:
            if(cloud.at(pt[0].x, pt[0].y).x == cloud.at(pt[0].x, pt[0].y).x && \
               cloud.at(pt[1].x, pt[1].y).x == cloud.at(pt[1].x, pt[1].y).x) {          // Check for nan
                out_msg.count = count;
                // Pole 1 Position
                out_msg.pole_1.x = cloud.at(pt[0].x, pt[0].y).x;
                out_msg.pole_1.y = cloud.at(pt[0].x, pt[0].y).y;
                out_msg.pole_1.z = cloud.at(pt[0].x, pt[0].y).z;
                // Pole 2 Position
                out_msg.pole_2.x = cloud.at(pt[1].x, pt[1].y).x;
                out_msg.pole_2.y = cloud.at(pt[1].x, pt[1].y).y;
                out_msg.pole_2.z = cloud.at(pt[1].x, pt[1].y).z;

                pole_pub.publish(out_msg);
            }
            break;
        case 3:
            if(cloud.at(pt[0].x, pt[0].y).x == cloud.at(pt[0].x, pt[0].y).x && \
               cloud.at(pt[1].x, pt[1].y).x == cloud.at(pt[1].x, pt[1].y).x && \
               cloud.at(pt[2].x, pt[2].y).x == cloud.at(pt[2].x, pt[2].y).x) {          // Check for nan
                out_msg.count = count;
                // Pole 1 Position
                out_msg.pole_1.x = cloud.at(pt[0].x, pt[0].y).x;
                out_msg.pole_1.y = cloud.at(pt[0].x, pt[0].y).y;
                out_msg.pole_1.z = cloud.at(pt[0].x, pt[0].y).z;
                // Pole 2 Position
                out_msg.pole_2.x = cloud.at(pt[1].x, pt[1].y).x;
                out_msg.pole_2.y = cloud.at(pt[1].x, pt[1].y).y;
                out_msg.pole_2.z = cloud.at(pt[1].x, pt[1].y).z;
                // Pole 3 Position
                out_msg.pole_3.x = cloud.at(pt[2].x, pt[2].y).x;
                out_msg.pole_3.y = cloud.at(pt[2].x, pt[2].y).y;
                out_msg.pole_3.z = cloud.at(pt[2].x, pt[2].y).z;

                pole_pub.publish(out_msg);
            }
            break;
        case 4:
            if(cloud.at(pt[0].x, pt[0].y).x == cloud.at(pt[0].x, pt[0].y).x && \
               cloud.at(pt[1].x, pt[1].y).x == cloud.at(pt[1].x, pt[1].y).x && \
               cloud.at(pt[2].x, pt[2].y).x == cloud.at(pt[2].x, pt[2].y).x && \
               cloud.at(pt[3].x, pt[3].y).x == cloud.at(pt[3].x, pt[3].y).x) {          // Check for nan
                out_msg.count = count;
                // Pole 1 Position
                out_msg.pole_1.x = cloud.at(pt[0].x, pt[0].y).x;
                out_msg.pole_1.y = cloud.at(pt[0].x, pt[0].y).y;
                out_msg.pole_1.z = cloud.at(pt[0].x, pt[0].y).z;
                // Pole 2 Position
                out_msg.pole_2.x = cloud.at(pt[1].x, pt[1].y).x;
                out_msg.pole_2.y = cloud.at(pt[1].x, pt[1].y).y;
                out_msg.pole_2.z = cloud.at(pt[1].x, pt[1].y).z;
                // Pole 3 Position
                out_msg.pole_3.x = cloud.at(pt[2].x, pt[2].y).x;
                out_msg.pole_3.y = cloud.at(pt[2].x, pt[2].y).y;
                out_msg.pole_3.z = cloud.at(pt[2].x, pt[2].y).z;
                // Pole 4 Position
                out_msg.pole_4.x = cloud.at(pt[3].x, pt[3].y).x;
                out_msg.pole_4.y = cloud.at(pt[3].x, pt[3].y).y;
                out_msg.pole_4.z = cloud.at(pt[3].x, pt[3].y).z;

                pole_pub.publish(out_msg);
            }
            break;
        case 5:
            if(cloud.at(pt[0].x, pt[0].y).x == cloud.at(pt[0].x, pt[0].y).x && \
               cloud.at(pt[1].x, pt[1].y).x == cloud.at(pt[1].x, pt[1].y).x && \
               cloud.at(pt[2].x, pt[2].y).x == cloud.at(pt[2].x, pt[2].y).x && \
               cloud.at(pt[3].x, pt[3].y).x == cloud.at(pt[3].x, pt[3].y).x && \
               cloud.at(pt[4].x, pt[4].y).x == cloud.at(pt[4].x, pt[4].y).x) {          // Check for nan
                out_msg.count = count;
                // Pole 1 Position
                out_msg.pole_1.x = cloud.at(pt[0].x, pt[0].y).x;
                out_msg.pole_1.y = cloud.at(pt[0].x, pt[0].y).y;
                out_msg.pole_1.z = cloud.at(pt[0].x, pt[0].y).z;
                // Pole 2 Position
                out_msg.pole_2.x = cloud.at(pt[1].x, pt[1].y).x;
                out_msg.pole_2.y = cloud.at(pt[1].x, pt[1].y).y;
                out_msg.pole_2.z = cloud.at(pt[1].x, pt[1].y).z;
                // Pole 3 Position
                out_msg.pole_3.x = cloud.at(pt[2].x, pt[2].y).x;
                out_msg.pole_3.y = cloud.at(pt[2].x, pt[2].y).y;
                out_msg.pole_3.z = cloud.at(pt[2].x, pt[2].y).z;
                // Pole 4 Position
                out_msg.pole_4.x = cloud.at(pt[3].x, pt[3].y).x;
                out_msg.pole_4.y = cloud.at(pt[3].x, pt[3].y).y;
                out_msg.pole_4.z = cloud.at(pt[3].x, pt[3].y).z;
                // Pole 5 Position
                out_msg.pole_5.x = cloud.at(pt[4].x, pt[4].y).x;
                out_msg.pole_5.y = cloud.at(pt[4].x, pt[4].y).y;
                out_msg.pole_5.z = cloud.at(pt[4].x, pt[4].y).z;

                pole_pub.publish(out_msg);
            }
            break;
        case 6:
            if(cloud.at(pt[0].x, pt[0].y).x == cloud.at(pt[0].x, pt[0].y).x && \
               cloud.at(pt[1].x, pt[1].y).x == cloud.at(pt[1].x, pt[1].y).x && \
               cloud.at(pt[2].x, pt[2].y).x == cloud.at(pt[2].x, pt[2].y).x && \
               cloud.at(pt[3].x, pt[3].y).x == cloud.at(pt[3].x, pt[3].y).x && \
               cloud.at(pt[4].x, pt[4].y).x == cloud.at(pt[4].x, pt[4].y).x && \
               cloud.at(pt[5].x, pt[5].y).x == cloud.at(pt[5].x, pt[5].y).x) {          // Check for nan
                out_msg.count = count;
                // Pole 1 Position
                out_msg.pole_1.x = cloud.at(pt[0].x, pt[0].y).x;
                out_msg.pole_1.y = cloud.at(pt[0].x, pt[0].y).y;
                out_msg.pole_1.z = cloud.at(pt[0].x, pt[0].y).z;
                // Pole 2 Position
                out_msg.pole_2.x = cloud.at(pt[1].x, pt[1].y).x;
                out_msg.pole_2.y = cloud.at(pt[1].x, pt[1].y).y;
                out_msg.pole_2.z = cloud.at(pt[1].x, pt[1].y).z;
                // Pole 3 Position
                out_msg.pole_3.x = cloud.at(pt[2].x, pt[2].y).x;
                out_msg.pole_3.y = cloud.at(pt[2].x, pt[2].y).y;
                out_msg.pole_3.z = cloud.at(pt[2].x, pt[2].y).z;
                // Pole 4 Position
                out_msg.pole_4.x = cloud.at(pt[3].x, pt[3].y).x;
                out_msg.pole_4.y = cloud.at(pt[3].x, pt[3].y).y;
                out_msg.pole_4.z = cloud.at(pt[3].x, pt[3].y).z;
                // Pole 5 Position
                out_msg.pole_5.x = cloud.at(pt[4].x, pt[4].y).x;
                out_msg.pole_5.y = cloud.at(pt[4].x, pt[4].y).y;
                out_msg.pole_5.z = cloud.at(pt[4].x, pt[4].y).z;
                // Pole 6 Position
                out_msg.pole_6.x = cloud.at(pt[5].x, pt[5].y).x;
                out_msg.pole_6.y = cloud.at(pt[5].x, pt[5].y).y;
                out_msg.pole_6.z = cloud.at(pt[5].x, pt[5].y).z;

                pole_pub.publish(out_msg);
            }
            break;
        case 7:
            if(cloud.at(pt[0].x, pt[0].y).x == cloud.at(pt[0].x, pt[0].y).x && \
               cloud.at(pt[1].x, pt[1].y).x == cloud.at(pt[1].x, pt[1].y).x && \
               cloud.at(pt[2].x, pt[2].y).x == cloud.at(pt[2].x, pt[2].y).x && \
               cloud.at(pt[3].x, pt[3].y).x == cloud.at(pt[3].x, pt[3].y).x && \
               cloud.at(pt[4].x, pt[4].y).x == cloud.at(pt[4].x, pt[4].y).x && \
               cloud.at(pt[5].x, pt[5].y).x == cloud.at(pt[5].x, pt[5].y).x && \
               cloud.at(pt[6].x, pt[6].y).x == cloud.at(pt[6].x, pt[6].y).x) {          // Check for nan
                out_msg.count = count;
                // Pole 1 Position
                out_msg.pole_1.x = cloud.at(pt[0].x, pt[0].y).x;
                out_msg.pole_1.y = cloud.at(pt[0].x, pt[0].y).y;
                out_msg.pole_1.z = cloud.at(pt[0].x, pt[0].y).z;
                // Pole 2 Position
                out_msg.pole_2.x = cloud.at(pt[1].x, pt[1].y).x;
                out_msg.pole_2.y = cloud.at(pt[1].x, pt[1].y).y;
                out_msg.pole_2.z = cloud.at(pt[1].x, pt[1].y).z;
                // Pole 3 Position
                out_msg.pole_3.x = cloud.at(pt[2].x, pt[2].y).x;
                out_msg.pole_3.y = cloud.at(pt[2].x, pt[2].y).y;
                out_msg.pole_3.z = cloud.at(pt[2].x, pt[2].y).z;
                // Pole 4 Position
                out_msg.pole_4.x = cloud.at(pt[3].x, pt[3].y).x;
                out_msg.pole_4.y = cloud.at(pt[3].x, pt[3].y).y;
                out_msg.pole_4.z = cloud.at(pt[3].x, pt[3].y).z;
                // Pole 5 Position
                out_msg.pole_5.x = cloud.at(pt[4].x, pt[4].y).x;
                out_msg.pole_5.y = cloud.at(pt[4].x, pt[4].y).y;
                out_msg.pole_5.z = cloud.at(pt[4].x, pt[4].y).z;
                // Pole 6 Position
                out_msg.pole_6.x = cloud.at(pt[5].x, pt[5].y).x;
                out_msg.pole_6.y = cloud.at(pt[5].x, pt[5].y).y;
                out_msg.pole_6.z = cloud.at(pt[5].x, pt[5].y).z;
                // Pole 7 Position
                out_msg.pole_7.x = cloud.at(pt[6].x, pt[6].y).x;
                out_msg.pole_7.y = cloud.at(pt[6].x, pt[6].y).y;
                out_msg.pole_7.z = cloud.at(pt[6].x, pt[6].y).z;

                pole_pub.publish(out_msg);
            }
            break;

    }
    PointCloud_Update=false;
}

void PoleDetection::showImages() {
    imshow("Cropped", imgCropped);
    waitKey(1);
    imshow("Original Image", originalImg);
    waitKey(1);
    imshow("Thresholded Image Yellow", imgThreshold_Green);
    waitKey(1);
    imshow( "RectangleYellow", RectangleGreen );
    waitKey(1);
}
