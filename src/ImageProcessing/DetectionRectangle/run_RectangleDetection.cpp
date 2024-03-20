/*
Bildverarbeitung des Kinect RGB Bildes mit OpenCV
Autor: Brandtner Philipp
*/
#include <ros/ros.h>
#include "RectangleDetection.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RectangleDetection");
  RectangleDetection rd;
  ros::spin();
  return 0;
}
