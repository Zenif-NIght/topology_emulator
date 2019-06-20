/**
 * @File: demo.cpp
 * @Date: 18 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Demos the use of the topic_rebroadcaster node.
 **/

/* Rebroadcaster_msgs Headers */
#include"rebroadcaster_msgs/ConnectTopics.h"
#include"rebroadcaster_msgs/DisconnectRebroadcast.h"

/* ROS Headers */
#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<std_msgs/String.h>

/* C++ Headers */
#include<chrono>
#include<thread>

void callback(const std_msgs::String& msg_in) { ROS_INFO("%s", msg_in.data.c_str()); }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "topic_rebroadcaster_demo_node");
  ros::NodeHandle m_nh;

  ros::ServiceClient connect_client    = m_nh.serviceClient<rebroadcaster_msgs::ConnectTopics>        ("topic_rebroadcaster/connect");
  ros::ServiceClient disconnect_client = m_nh.serviceClient<rebroadcaster_msgs::DisconnectRebroadcast>("topic_rebroadcaster/disconnect");

  ros::Publisher pub1 = m_nh.advertise<std_msgs::String>("outTopic1", 5);
  ros::Publisher pub2 = m_nh.advertise<std_msgs::String>("outTopic2", 5);
  ros::Publisher pub3 = m_nh.advertise<std_msgs::String>("outTopic3", 5);
  
  ros::Subscriber sub = m_nh.subscribe("inTopic", 15, callback);

  // Connect topics
  rebroadcaster_msgs::ConnectTopics connect_obj;
  connect_obj.request.topic_out = "inTopic";  
  connect_obj.request.spin_rate = 1;
  connect_obj.request.queue_length = 5;

  connect_obj.request.topic_in = "outTopic1";
  connect_client.call(connect_obj);
 
  connect_obj.request.topic_in = "outTopic2";
  connect_client.call(connect_obj);

  connect_obj.request.topic_in = "outTopic3";
  connect_client.call(connect_obj);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  ROS_INFO("Topics connected");

  // Publish messages
  std_msgs::String msg;
  msg.data = "Now";
  pub1.publish(msg);
  msg.data = "I";
  pub2.publish(msg);
  msg.data = "Can";
  pub3.publish(msg);
  msg.data = "Use";
  pub1.publish(msg);
  msg.data = "Any";
  pub2.publish(msg);
  msg.data = "Of";
  pub3.publish(msg);
  msg.data = "Them!";
  pub1.publish(msg);

  // Wait for the messages to come back
  ros::Time timer = ros::Time::now();
  ros::Rate loop_rate(10);

  while(ros::ok() && (ros::Duration(3) > (ros::Time::now() - timer)))
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Close topics
  rebroadcaster_msgs::DisconnectRebroadcast disconnect_obj;
  disconnect_obj.request.topic_out = "inTopic";

  disconnect_obj.request.topic_in = "outTopic1";
  disconnect_client.call(disconnect_obj);

  disconnect_obj.request.topic_in = "outTopic2";
  disconnect_client.call(disconnect_obj);

  disconnect_obj.request.topic_in = "outTopic3";
  disconnect_client.call(disconnect_obj);

  ROS_INFO("Topics disconnected");

  // Make sure the topics aren't connected anymore
  msg.data = "Now";
  pub1.publish(msg);
  msg.data = "I";
  pub2.publish(msg);
  msg.data = "Can't";
  pub3.publish(msg);
  msg.data = "Use";
  pub1.publish(msg);
  msg.data = "Any";
  pub2.publish(msg);
  msg.data = "Of";
  pub3.publish(msg);
  msg.data = "Them!";
  pub1.publish(msg);

  timer = ros::Time::now();

  while(ros::ok() && (ros::Duration(3) > (ros::Time::now() - timer)))
  {
    ros::spinOnce();
  }

  exit(EXIT_SUCCESS);
}

/* demo.cpp */

