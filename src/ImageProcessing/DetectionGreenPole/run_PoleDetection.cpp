/*
Bildverarbeitung des Kinect RGB Bildes mit OpenCV
Autor: Brandtner Philipp
*/
#include <ros/ros.h>
#include "PoleDetection.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PoleDetectionGreen");
  PoleDetection pd;
  ros::spin();
  return 0;
}
