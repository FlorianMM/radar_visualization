/************************************************************************
Description: Node for pseudo random generated RadarDistance messages

Author: Florian Müller-Martin (florian.mueller-martin@siemens-healthineers.com)

Date: 29.07.2022
*************************************************************************/

#include "ros/ros.h"
#include "radar_visualization/RadarDistance.h"

// global variables
uint16_t msg_id = 0;

// function prototypes
radar_visualization::RadarDistance createMessage(uint32_t sensorID);

int main(int argc, char** argv)
{
  // init
  ros::init(argc, argv, "MessageGenerationNode");
  ros::NodeHandle n;

  ros::Publisher RadarData_pub = n.advertise<radar_visualization::RadarDistance>("RadarData", 1000);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    radar_visualization::RadarDistance msg = createMessage(1);

    RadarData_pub.publish(msg);
    ROS_INFO("Send message with ID: %d", msg.header.seq);
    loop_rate.sleep();
  }

  return 0;
}

/**
 * @brief Create a message Object containing a current timestamp and random values in DistanceBins[]
 * @param sensorID ID of the sending sensor
 * @return radar_visualization::RadarDistance Filled message Object
 */
radar_visualization::RadarDistance createMessage(uint32_t sensorID)
{
  radar_visualization::RadarDistance msg;
  msg.header.seq = msg_id++;
  msg.header.stamp = ros::Time::now();
  msg.sampleTime = ros::Time::now();
  msg.sensorID = 1;
  msg.fovHorizontal = 1.3962634015955;
  msg.fovVertical = 0.69813170079773;
  msg.numbersOfBins = 1024;
  msg.startDistance = 0.05;
  msg.endDistance = 2;
  msg.lengthOfBin = (msg.endDistance - msg.startDistance) / msg.numbersOfBins;

  std::vector<uint32_t> rand_DistanceBins;
  for (int i = 0; i < msg.numbersOfBins; i++)
  {
    rand_DistanceBins.push_back(rand() % 12000);
  }
  msg.DistanceBins = rand_DistanceBins;

  return msg;
}
