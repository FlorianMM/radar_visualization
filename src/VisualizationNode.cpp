/************************************************************************
Description: Node for visualizing RadarDistance messages

Author: Florian Müller-Martin (florian.mueller-martin@siemens-healthineers.com)

Date: 29.07.2022
*************************************************************************/

#include "ros/ros.h"
#include "radar_visualization/RadarDistance.h"

void RadarDataCallback(const radar_visualization::RadarDistance msg)
{
  ROS_INFO("I received a message with ID: %d", msg.header.seq);
}

int main(int argc, char** argv)
{
  // init
  ros::init(argc, argv, "VisualizationNode");
  ros::NodeHandle n;

  ros::Subscriber RadarData_sub = n.subscribe("RadarData", 1000, RadarDataCallback);

  ros::spin();

  return 0;
}
