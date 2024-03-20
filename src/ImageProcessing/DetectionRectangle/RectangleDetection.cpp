#include "RectangleDetection.h"

/*
 * Constructor for RectangleDetection Class
 */
RectangleDetection::RectangleDetection() : it_(nh_)
{
    countBlueRect = 0;
    countYellowRect = 0;
    // Initializing of HSV values for each color

    HSV_yellow.iLowH = 0;
    HSV_yellow.iHighH = 57;
    HSV_yellow.iLowS = 0;
    HSV_yellow.iHighS = 215;
    HSV_yellow.iLowV = 251;
    HSV_yellow.iHighV = 255;

    HSV_blue.iLowH = 100;
    HSV_blue.iHighH = 139;
    HSV_blue.iLowS = 20;
    HSV_blue.iHighS = 195;
    HSV_blue.iLowV = 60;
    HSV_blue.iHighV = 255;

    createControlbar();

    // Subscrive to input video feed
    Image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1, &RectangleDetection::loadImage, this);
    // Subscribe to service feed
    srvRectangleDetection = nh_.advertiseService("srvRectangleDetection", &RectangleDetection::SrvCall,this);

}

/*
 * Destructor for RectangleDetection Class
 */

RectangleDetection::~RectangleDetection()
{
    ROS_INFO("DESTROY");
    destroyWindow(OPENCV_WINDOW);
}

/*
 * Create Treckbars for HSV value adjustment
 */

void RectangleDetection::createControlbar()
{
    namedWindow("Control Yellow", CV_WINDOW_AUTOSIZE);
    createTrackbar("LowH", "Control Yellow", &HSV_yellow.iLowH, 179); //Hue (0 - 179)
    createTrackbar("HighH", "Control Yellow", &HSV_yellow.iHighH, 179);

    createTrackbar("LowS", "Control Yellow", &HSV_yellow.iLowS, 255); //Saturation (0 - 255)
    createTrackbar("HighS", "Control Yellow", &HSV_yellow.iHighS, 255);

    createTrackbar("LowV", "Control Yellow", &HSV_yellow.iLowV, 255); //Value (0 - 255)
    createTrackbar("HighV", "Control Yellow", &HSV_yellow.iHighV, 255);

    namedWindow("Control Blue", CV_WINDOW_AUTOSIZE);
    createTrackbar("LowH", "Control Blue", &HSV_blue.iLowH, 179); //Hue (0 - 179)
    createTrackbar("HighH", "Control Blue", &HSV_blue.iHighH, 179);

    createTrackbar("LowS", "Control Blue", &HSV_blue.iLowS, 255); //Saturation (0 - 255)
    createTrackbar("HighS", "Control Blue", &HSV_blue.iHighS, 255);

    createTrackbar("LowV", "Control Blue", &HSV_blue.iLowV, 255); //Value (0 - 255)
    createTrackbar("HighV", "Control Blue", &HSV_blue.iHighV, 255);
}

/*
 * Load Images from RBG Topic
 */

void RectangleDetection::loadImage(const sensor_msgs::ImageConstPtr& msg) {
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
    namedWindow(OPENCV_WINDOW);

    // Filter each color of the Image
    CropImage();
}

void RectangleDetection::CropImage() {
    Mat test;
    Rect croppedRectangle = Rect(0,originalImg.rows/2,originalImg.cols,originalImg.rows/2);
    imgCropped = originalImg(croppedRectangle);

    imshow("Cropped", imgCropped);
    waitKey(1);
    ColorFilter();
}

/*
 * Image processing for RBG picture
 * - Convert RGB picture to HSV and apply Filter
 * - Do morphological opening and closing to remove Noise
 */

void RectangleDetection::ColorFilter()
{

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
    cvtColor(imgCropped, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    int value=25;
    /************************* BLUE *************************/
    //Threshold the images with the color blue
    inRange(imgHSV, Scalar(HSV_blue.iLowH, HSV_blue.iLowS, HSV_blue.iLowV), Scalar(HSV_blue.iHighH, HSV_blue.iHighS, HSV_blue.iHighV), imgThreshold_Blue);
    //morphological opening (remove small objects from the foreground)
    erode(imgThreshold_Blue, imgThreshold_Blue, getStructuringElement(MORPH_ELLIPSE, Size(value, value)) );
    dilate( imgThreshold_Blue, imgThreshold_Blue, getStructuringElement(MORPH_ELLIPSE, Size(value, value)) );

    //morphological closing (fill small holes in the foreground)
    dilate( imgThreshold_Blue, imgThreshold_Blue, getStructuringElement(MORPH_ELLIPSE, Size(value,value)) );
    erode(imgThreshold_Blue, imgThreshold_Blue, getStructuringElement(MORPH_ELLIPSE, Size(value, value)) );
    //imshow("Thresholded Image Green no smooth", imgThreshold_Green); //show the thresholded image


    /************************* YELLOW *************************/
    //Threshold the images with the color yellow
    inRange(imgHSV, Scalar(HSV_yellow.iLowH, HSV_yellow.iLowS, HSV_yellow.iLowV), Scalar(HSV_yellow.iHighH, HSV_yellow.iHighS, HSV_yellow.iHighV), imgThreshold_Yellow);

    //morphological opening (remove small objects from the foreground)
    erode(imgThreshold_Yellow, imgThreshold_Yellow, getStructuringElement(MORPH_ELLIPSE, Size(value, value)) );
    dilate( imgThreshold_Yellow, imgThreshold_Yellow, getStructuringElement(MORPH_ELLIPSE, Size(value, value)) );

    //morphological closing (fill small holes in the foreground)
    dilate( imgThreshold_Yellow, imgThreshold_Yellow, getStructuringElement(MORPH_ELLIPSE, Size(value,value)) );
    erode(imgThreshold_Yellow, imgThreshold_Yellow, getStructuringElement(MORPH_ELLIPSE, Size(value, value)) );
    //imshow("Thresholded Image Green no smooth", imgThreshold_Green); //show the thresholded image

    // Show the images
    imshow(OPENCV_WINDOW, originalImg);
    waitKey(1);
    imshow("Thresholded Image Blue", imgThreshold_Blue); //show the thresholded image
    waitKey(1);
    imshow("Thresholded Image Yellow", imgThreshold_Yellow); //show the thresholded image
    waitKey(1);

    updateBlue = false;
    updateYellow = false;
    RectangleApproxBlue();
    RectangleApproxYellow();
}


/*
 * Rectangle Detection Blue
 */

void RectangleDetection::RectangleApproxBlue()
{
    double ratio, area;
    int thresh = 100;
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Detect edges using Threshold
    //threshold( imgThreshold_Yellow, threshold_output, thresh, 255, THRESH_BINARY );
    //imshow("thres",threshold_output);
    //waitKey(30);
    /// Find contours
    findContours( imgThreshold_Blue, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );

    /// Approximate contours to polygons + get bounding rects
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    int count = contours.size();


    /// Find the rotated rectangles and ellipses for each contour
    vector<RotatedRect> minRect( contours.size() );
    int k = 0;
    for( int i = 0; i < count; i++ ) {
        minRect[i] = minAreaRect( Mat(contours[k]) );
        //ROS_INFO("Height: %f",minRect[i].size.height);
        //ROS_INFO("Width: %f",minRect[i].size.width);
        //ROS_INFO("Angle: %f", minRect[i].angle);
        area = (double)((double)minRect[i].size.height*(double)minRect[i].size.width);
        //ROS_INFO("Area: %f",area);

        // First filtering to get rid of small contour areas and areas, which don't fullfill the ratio ... ratio 8.75
        if(area < 15000.0) { //|| (double)minRect[i].width > 450.0 || (double)minRect[i].height < 70.0) { //ratio < 7.0
            i--;
            count--;
        }
        k++;
    }
    areaBlue = (double)((double)minRect[0].size.height*(double)minRect[0].size.width);
    //ROS_INFO("Count: %i", count);
    /// Draw contours + rotated rects + ellipses
    Mat drawing = Mat::zeros( imgThreshold_Blue.size(), CV_8UC3 );
    for( int i = 0; i < count; i++ ) {
        Scalar color = Scalar( 0, 255, 140 );
        // rotated rectangle
        Point2f rect_points[4]; minRect[i].points( rect_points );
        for( int j = 0; j < 4; j++ )
          line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
    }

    if (updateBlueRect==1 || updateBlueRect==0 ) {
        updateBlue=true;
        decideColor();
    }

    /// Show in a window
    imshow( "RectangleBlue", drawing );
    waitKey(1);
}

/*
 * Rectangle Detection Yellow
 */

void RectangleDetection::RectangleApproxYellow()
{
    double ratio, area;
    int thresh = 100;
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Detect edges using Threshold
    //threshold( imgThreshold_Yellow, threshold_output, thresh, 255, THRESH_BINARY );
    //imshow("thres",threshold_output);
    //waitKey(30);
    /// Find contours
    findContours( imgThreshold_Yellow, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );

    /// Approximate contours to polygons + get bounding rects
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    int count = contours.size();


    /// Find the rotated rectangles and ellipses for each contour
    vector<RotatedRect> minRect( contours.size() );

    int k = 0;
    for( int i = 0; i < count; i++ ) {
        minRect[i] = minAreaRect( Mat(contours[k]) );
        /*
        ROS_INFO("Height: %f",minRect[i].size.height);
        ROS_INFO("Width: %f",minRect[i].size.width);
        ROS_INFO("Angle: %f", minRect[i].angle);
        */
        area = (double)((double)minRect[i].size.height*(double)minRect[i].size.width);
        //ROS_INFO("Area: %f",area);

        // First filtering to get rid of small contour areas and areas, which don't fullfill the ratio ... ratio 8.75

        if(area < 30000.0) { //|| (double)minRect[i].width > 450.0 || (double)minRect[i].height < 70.0) { //ratio < 7.0
            i--;
            count--;
        }
        k++;
    }
    areaYellow = (double)((double)minRect[0].size.height*(double)minRect[0].size.width);
    updateYellowRect = count;
    //ROS_INFO("PresentCount: %i", count);
    /// Draw contours + rotated rects + ellipses
    Mat drawing = Mat::zeros( imgThreshold_Yellow.size(), CV_8UC3 );
    for( int i = 0; i < count; i++ ) {
        Scalar color = Scalar( 0, 255, 140 );
        // rotated rectangle
        Point2f rect_points[4]; minRect[i].points( rect_points );
        for( int j = 0; j < 4; j++ )
          line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
    }
    /// Show in a window
    imshow( "YellowRectangle", drawing );
    waitKey(1);

    if (updateYellowRect==1 || updateYellowRect ==0 ) {
        updateYellow=true;
        decideColor();
    }
}

void RectangleDetection::decideColor() {
    if(updateYellow && updateBlue) {
        if(updateYellowRect==1 && updateBlueRect==1) {
            if(areaYellow > areaBlue)
                countYellowRect++;
            else
                countBlueRect++;
        }
        else {
            if(updateYellowRect == 1)
                countYellowRect++;
            if(updateBlueRect == 1)
                countBlueRect++;
        }
        ROS_INFO("CountBlue: %i",countBlueRect);
        ROS_INFO("CountYellow: %i",countYellowRect);
    }
}

/*
 *  Service Call from Master for RectangleDetection
 */

bool RectangleDetection::SrvCall(gruppe6::srvRectangleDetection::Request  &req, gruppe6::srvRectangleDetection::Response &res)
{
    if(countBlueRect>=countYellowRect)
        res.TeamColor = false;
    else
        res.TeamColor = true;

    return true;
}
