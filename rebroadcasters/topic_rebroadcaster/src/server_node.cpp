/**
 * @File: server_node.cpp
 * @Date: 18 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Node to run topic rebroadcasting server.
 **/

/* Local Headers */
#include"topic_rebroadcaster/relay_server.hpp"

/* ROS Headers */
#include<ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "topic_rebroadcaster_node");
  ros::NodeHandle m_nh;

  RelayServer server;

  ros::Rate loop_rate(1);

  while(m_nh.ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  exit(EXIT_SUCCESS);
}

/* server_node.cpp */

