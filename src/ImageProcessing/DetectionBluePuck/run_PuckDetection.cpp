/*
Bildverarbeitung des Kinect RGB Bildes mit OpenCV
Autor: Brandtner Philipp
*/
#include <ros/ros.h>
#include "PuckDetection.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PuckDetectionBlue");
  PuckDetection pd;
  ros::spin();
  return 0;
}
