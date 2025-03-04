#include "PuckDetection.h"

/*
 * Constructor for PuckDetection Class
 */
PuckDetection::PuckDetection() : it_(nh_)
{
    ROS_INFO("PUCKDETECTION YELLOW");
    ROS_INFO("--> Running!");
    ROS_INFO("Echo topic for data.");
    PointCloud_Update = false;
    // Initializing of HSV values for each color

    HSV_yellow.iLowH = 16;
    HSV_yellow.iHighH = 38;
    HSV_yellow.iLowS = 41;
    HSV_yellow.iHighS = 255;
    HSV_yellow.iLowV = 171;
    HSV_yellow.iHighV = 255;

    createControlbar();

    // Subscrive to input video feed
    //image_sub2_ = it_.subscribe("/camera/depth/image", 1, &PuckDetection::loadImageDepth, this);
    Image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1, &PuckDetection::loadImage, this);
    PointCloud_sub = nh_.subscribe("/camera/depth/points", 1, &PuckDetection::loadPointCloud, this);
    //image_pub_ = it_.advertise("/puck_detection/blue", 1);
    puck_pub = nh_.advertise<gruppe6::Puck>("/puck_detection/yellow", 1000);
    filteredImage_pub = nh_.advertise<sensor_msgs::Image>("Image/YellowPuck", 1000);
}

/*
 * Destructor for PuckDetection Class
 */

PuckDetection::~PuckDetection()
{
    ROS_INFO("DESTROY");
    destroyWindow(OPENCV_WINDOW);
}

/*
 * Create Treckbars for HSV value adjustment
 */

void PuckDetection::createControlbar()
{
    namedWindow("Control Yellow", CV_WINDOW_AUTOSIZE);
    createTrackbar("LowH", "Control Yellow", &HSV_yellow.iLowH, 179); //Hue (0 - 179)
    createTrackbar("HighH", "Control Yellow", &HSV_yellow.iHighH, 179);

    createTrackbar("LowS", "Control Yellow", &HSV_yellow.iLowS, 255); //Saturation (0 - 255)
    createTrackbar("HighS", "Control Yellow", &HSV_yellow.iHighS, 255);

    createTrackbar("LowV", "Control Yellow", &HSV_yellow.iLowV, 255); //Value (0 - 255)
    createTrackbar("HighV", "Control Yellow", &HSV_yellow.iHighV, 255);
}

/*
 * Load Images from RBG Topic
 */

void PuckDetection::loadImage(const sensor_msgs::ImageConstPtr& msg) {
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
    ColorFilter();
}

/*
 * Load Pointcloud from Pointcloud Topic
 */

void PuckDetection::loadPointCloud(const sensor_msgs::PointCloud2ConstPtr& input) {
    PointCloud_Update=true;

    // Do data processing here...
    //ROS_INFO("GOT CLOUD DATA");
    PointCloud_Input=*input;
}


void PuckDetection::CropImage() {
    Rect croppedRectangle = Rect(0,originalImg.rows/2,originalImg.cols,originalImg.rows/2);
    imgCropped = imgThreshold_Yellow(croppedRectangle);
    RectangleApprox();
}

/*
 * Image processing for RBG picture
 * - Convert RGB picture to HSV and apply Filter
 * - Do morphological opening and closing to remove Noise
 */

void PuckDetection::ColorFilter()
{
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
    cvtColor(originalImg, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    //Threshold the images with the color
    inRange(imgHSV, Scalar(HSV_yellow.iLowH, HSV_yellow.iLowS, HSV_yellow.iLowV), Scalar(HSV_yellow.iHighH, HSV_yellow.iHighS, HSV_yellow.iHighV), imgThreshold_Yellow);

    /************************* Yellow *************************/
    int factor = 10;
    //morphological opening (remove small objects from the foreground)
    erode(imgThreshold_Yellow, imgThreshold_Yellow, getStructuringElement(MORPH_ELLIPSE, Size(factor, factor)) );
    dilate( imgThreshold_Yellow, imgThreshold_Yellow, getStructuringElement(MORPH_ELLIPSE, Size(factor, factor)) );

    //morphological closing (fill small holes in the foreground)
    dilate( imgThreshold_Yellow, imgThreshold_Yellow, getStructuringElement(MORPH_ELLIPSE, Size(factor,factor)) );
    erode(imgThreshold_Yellow, imgThreshold_Yellow, getStructuringElement(MORPH_ELLIPSE, Size(factor, factor)) );

    imshow("Thresholded Image Yellow", imgThreshold_Yellow);
    waitKey(1);
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, imgThreshold_Yellow);
    img_bridge.toImageMsg(img_msg);
    filteredImage_pub.publish(img_msg);
    CropImage();
    
}


//durchmesser=12.5
//länge=13.5+22.5
//Verhältnis=2.88

/*
 * Rectangle approximation
 */

void PuckDetection::RectangleApprox()
{
    double ratio, area;
    int thresh = 100;
    Mat contoursInput=imgCropped;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    //threshold( imgThreshold_Yellow, threshold_output, thresh, 255, THRESH_BINARY );
    /// Find contours
    findContours( contoursInput, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );


    /// Approximate contours to polygons + get bounding rects
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    int count = contours.size();
    geometry_msgs::Point pt[3];

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
        ROS_INFO("Area: %f", area);
        ///ROS_INFO("CenterX: %f",minRect[i].center.x);
        //ROS_INFO("CenterY: %f",minRect[i].center.y);
        // First filtering to get rid of small contour areas and areas, which don't fullfill the ratio ... ratio 8.75
        if(area < 250.0 || ratio > 3.1 || ratio < 0.9) { //|| (double)minRect[i].width > 450.0 || (double)minRect[i].height < 70.0) { //ratio < 7.0
            i--;
            count--;
        }
        else {
            pt[i].x = minRect[i].center.x;
            pt[i].y = minRect[i].center.y + 120;
            pt[i].z=count;
        }
        k++;
    }

    /// Draw contours + rotated rects + ellipses
/*
    RectangleYellow = Mat::zeros( imgCropped.size(), CV_8UC3 );
    for( int i = 0; i < count; i++ ) {
        Scalar color = Scalar( 0, 255, 140 );
        // rotated rectangle
        Point2f rect_points[4]; minRect[i].points( rect_points );
        for( int j = 0; j < 4; j++ )
          line( RectangleYellow, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
    }
*/
    Point pt_test;
    pt_test.x=640;
    pt_test.y=240;
    
    Scalar color = Scalar( 0, 140, 140 );
    circle(RectangleYellow, pt_test,5,color,2, 8, 0);

    //showImages();
    if(PointCloud_Update) 
        PointCloudProcessing(pt, count);

}

void PuckDetection::PointCloudProcessing(geometry_msgs::Point pt[], int count) {
    float distance[3] = {0.0, 0.0, 0.0};
    gruppe6::Puck out_msg;
    // Container for original & filtered data
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(PointCloud_Input, cloud);
    //std::cout << "(width:) = " << (int) cloud.width << std::endl;
    //std::cout << "(height:) = " << (int) cloud.height << std::endl;
    switch(count) {
        case 0:
                CreatOutMsg(pt,count,distance, cloud);
            break;
        case 1:
            //std::cout << "(First:) = " << cloud.at((int) round(pt[0].x),(int) round(pt[0].y)) << std::endl;
            if(cloud.at(pt[0].x, pt[0].y).x == cloud.at(pt[0].x, pt[0].y).x)          // Check for nan
                    CreatOutMsg(pt,count,distance, cloud);
            break;
        case 2:
            //std::cout << "(First:) = " << cloud.at(pt[0].x, pt[0].y) << std::endl;
            //std::cout << "(Second:) = " << cloud.at(pt[1].x, pt[1].y) << std::endl;
            distance[0] = pcl::euclideanDistance(cloud.at(pt[0].x,pt[0].y),cloud.at(pt[1].x,pt[1].y));

            if(cloud.at(pt[0].x, pt[0].y).x == cloud.at(pt[0].x, pt[0].y).x && \
               cloud.at(pt[1].x, pt[1].y).x == cloud.at(pt[1].x, pt[1].y).x && \
               distance[0] == distance[0])          // Check for nan
                    CreatOutMsg(pt,count,distance, cloud);

            //std::cout << "(Distance first and second:) = " << distance << std::endl;
            break;
        case 3:
            //std::cout << "(First:) = " << cloud.at(pt[0].x, pt[0].y) << std::endl;
            //std::cout << "(Second:) = " << cloud.at(pt[1].x, pt[1].y) << std::endl;
            //std::cout << "(Third:) = " << cloud.at(pt[2].x, pt[2].y) << std::endl;
            distance[0] = pcl::euclideanDistance(cloud.at(pt[0].x,pt[0].y),cloud.at(pt[1].x,pt[1].y));
            //std::cout << "(Distance first and second:) = " << distance[0] << std::endl;
            distance[1] = pcl::euclideanDistance(cloud.at(pt[0].x,pt[0].y),cloud.at(pt[2].x,pt[2].y));
            //std::cout << "(Distance first and third:) = " << distance[1] << std::endl;
            distance[2] = pcl::euclideanDistance(cloud.at(pt[1].x,pt[1].y),cloud.at(pt[2].x,pt[2].y));
            //std::cout << "(Distance second and third:) = " << distance[2] << std::endl;
            if(cloud.at(pt[0].x, pt[0].y).x == cloud.at(pt[0].x, pt[0].y).x && \
               cloud.at(pt[1].x, pt[1].y).x == cloud.at(pt[1].x, pt[1].y).x && \
               cloud.at(pt[2].x, pt[2].y).x == cloud.at(pt[2].x, pt[2].y).x && \
               distance[0] == distance[0] && distance[1] == distance[1] && distance[2] == distance[2])          // Check for nan
                    CreatOutMsg(pt,count,distance, cloud);
            break;
        default:
            ROS_INFO("too many detected...");
    }
    PointCloud_Update=false;
}

void PuckDetection::CreatOutMsg(geometry_msgs::Point pt[], int count, float distance[3], pcl::PointCloud<pcl::PointXYZ> cloud) {
    gruppe6::Puck out_msg;

    switch(count) {
        case 0:
            out_msg.count = 0;
            break;
        case 1:
            out_msg.count = 1;
            // Puck 1 Position
            out_msg.puck_1.x = cloud.at(pt[0].x, pt[0].y).x;
            out_msg.puck_1.y = cloud.at(pt[0].x, pt[0].y).y;
            out_msg.puck_1.z = cloud.at(pt[0].x, pt[0].y).z;
            break;
        case 2:
            out_msg.count = 2;
            // Puck 1 Position
            out_msg.puck_1.x = cloud.at(pt[0].x, pt[0].y).x;
            out_msg.puck_1.y = cloud.at(pt[0].x, pt[0].y).y;
            out_msg.puck_1.z = cloud.at(pt[0].x, pt[0].y).z;
            // Puck 2 Position
            out_msg.puck_2.x = cloud.at(pt[1].x, pt[1].y).x;
            out_msg.puck_2.y = cloud.at(pt[1].x, pt[1].y).y;
            out_msg.puck_2.z = cloud.at(pt[1].x, pt[1].y).z;
            // Distance between Puck 1 and Puck 2
            out_msg.dist_12 = distance[0];
            break;
        case 3:
            out_msg.count = 3;
            // Puck 1 Position
            out_msg.puck_1.x = cloud.at(pt[0].x, pt[0].y).x;
            out_msg.puck_1.y = cloud.at(pt[0].x, pt[0].y).y;
            out_msg.puck_1.z = cloud.at(pt[0].x, pt[0].y).z;
            // Puck 2 Position
            out_msg.puck_2.x = cloud.at(pt[1].x, pt[1].y).x;
            out_msg.puck_2.y = cloud.at(pt[1].x, pt[1].y).y;
            out_msg.puck_2.z = cloud.at(pt[1].x, pt[1].y).z;
            // Puck 3 Position
            out_msg.puck_3.x = cloud.at(pt[2].x, pt[2].y).x;
            out_msg.puck_3.y = cloud.at(pt[2].x, pt[2].y).y;
            out_msg.puck_3.z = cloud.at(pt[2].x, pt[2].y).z;
            // Distance between Puck 1 and Puck 2
            out_msg.dist_12 = distance[0];
            // Distance between Puck 1 and Puck 3
            out_msg.dist_13 = distance[1];
            // Distance between Puck 2 and Puck 2
            out_msg.dist_23 = distance[2];
            break;
    }
    puck_pub.publish(out_msg);
    PointCloud_Update=false;
}

void PuckDetection::showImages() {
    imshow("Cropped", imgCropped);
    waitKey(1);
    imshow("Original Image", originalImg);
    waitKey(1);
    
    /// Show in a window
    imshow( "RectangleYellow", RectangleYellow );
    waitKey(1);
}
