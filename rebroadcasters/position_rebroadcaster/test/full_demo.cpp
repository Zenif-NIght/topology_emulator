/**
 * @File: full_demo.cpp
 * @Date: 17 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Simply opens a topic to get pose info from and prints that info as it
 * comes in.
 **/

/* Rebroadcaster Msgs Headers */
#include"rebroadcaster_msgs/ConnectPositionServer.h"
#include"rebroadcaster_msgs/DisconnectRebroadcast.h"

/* Multi Vehicle Headers */
#include<mv_msgs/VehiclePoses.h>

/* ROS Headers */
#include<ros/ros.h>
#include<ros/callback_queue.h>

/* C++ Headers */
#include<chrono>
#include<thread>

void callback(const mv_msgs::VehiclePoses& msg) { ROS_INFO_STREAM(msg); };

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rebroadcaster_demo_node");
  ros::NodeHandle m_nh;
  
  ROS_INFO("Start of demo");

  ros::ServiceClient srv_client = m_nh.serviceClient<rebroadcaster_msgs::ConnectPositionServer>("position_rebroadcaster/connect");
  ros::ServiceClient end_client = m_nh.serviceClient<rebroadcaster_msgs::DisconnectRebroadcast>("position_rebroadcaster/disconnect");
  ros::Subscriber    msg_sub    = m_nh.subscribe("to_full_demo", 5, callback);

  ROS_INFO("Opening a topic");

  // Open topic
  rebroadcaster_msgs::ConnectPositionServer srv_obj;
  srv_obj.request.topic = "to_full_demo";
  srv_obj.request.frameId = "rosbot1";

  if(!srv_client.call(srv_obj))
  {
    ROS_ERROR("Service Failed");
  }

  ROS_INFO("Printing messages");

  ros::Rate loop_rate(0.5);
  ros::Time timer = ros::Time::now();
  
  // Print messages
  while(m_nh.ok() && (ros::Duration(10) > (ros::Time::now() - timer)))
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("Closing topic");

  // Close topic
  rebroadcaster_msgs::DisconnectRebroadcast end_obj;
  srv_obj.request.topic = "to_full_demo";

  if(!end_client.call(end_obj))
  {
    ROS_ERROR("End Service Failed");
  }

  ROS_INFO("Checking closed topic");

  // Make sure messages aren't being sent
  ros::getGlobalCallbackQueue()->clear();

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  ros::getGlobalCallbackQueue()->clear();

  timer = ros::Time::now();

  ROS_INFO("There should be no more messages!");

  while(m_nh.ok() && (ros::Duration(3) > (ros::Time::now() - timer)))
  {
    ros::spinOnce();
  }

  ROS_INFO("End of demo");

  exit(EXIT_SUCCESS);
}

/* full_demo.cpp */

