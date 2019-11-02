/**
 * @File: position_rebroadcaster_node.cpp
 * @Date: 17 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * This node provides service calls to republish position data of simulated
 * turtlebots.
 **/

/* Local Headers */
#include"position_rebroadcasters/agent_pool.hpp"
#include"position_rebroadcasters/output_server.hpp"

/* ROS Headers */
#include<ros/ros.h>

/* C++ Headers */
#include<string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_rebroadcaster_node");
  ros::NodeHandle p_nh("~");

  std::string network_topology_topic = std::string();
  int agent_discovery_spin_rate      = 0;
  int agent_callback_queue_length    = 0;
  int agent_refresh_rate             = 0;
  int publishers_queue_length        = 0;
  int publishing_spin_rate           = 0;
  int server_responce_speed          = 0;

  p_nh.getParam("network_topology_topic",      network_topology_topic);
  p_nh.param   ("agent_discovery_spin_rate",   agent_discovery_spin_rate,   1);
  p_nh.param   ("agent_callback_queue_length", agent_callback_queue_length, 5);
  p_nh.param   ("agent_refresh_rate",          agent_refresh_rate,          30);
  p_nh.param   ("publishers_queue_length",     publishers_queue_length,     5);
  p_nh.param   ("publishing_spin_rate",        publishing_spin_rate,        30);
  p_nh.param   ("server_responce_speed",       server_responce_speed,       1);

  OutputServer server(network_topology_topic,
                      agent_discovery_spin_rate,
                      agent_callback_queue_length,
                      agent_refresh_rate,
                      publishers_queue_length,
                      publishing_spin_rate);

  ros::Rate loop_rate(server_responce_speed);

  while(p_nh.ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  exit(EXIT_SUCCESS);
}

/* position_rebroadcaster_node.cpp */

