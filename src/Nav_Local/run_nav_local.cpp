/*
Navigation im Nahfeld um den Puck
Autor: Simon Wölzmüller
*/
#include <ros/ros.h>
#include "nav_local.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Local_Navigation");
  Local_Navigation c;
  ros::spin();
  return 0;
}
