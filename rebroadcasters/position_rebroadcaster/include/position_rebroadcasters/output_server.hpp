/**
 * @File: output_server.hpp
 * @Date: 16 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * This class is made to oversee the creation and destruction of vehicle position
 * message topics.
 **/

#ifndef OUTPUT_SERVER_HPP
#define OUTPUT_SERVER_HPP

/* Local Headers */
#include"position_rebroadcasters/agent_pool.hpp"
#include"position_rebroadcasters/position_publisher.hpp"

/* rebroadcaste_msgs Headers */
//#include<rebroadcaste_msgs/ConnectPositionServer.h>
//#include<rebroadcaste_msgs/DisconnectRebroadcast.h>

/* ROS Headers */
#include<ros/ros.h>

/* C++ Headers */
#include<map>
#include<string>

class OutputServer 
{
public:
  /**
   * @Default Constructor
   **/
  OutputServer() = delete;
  /**
   * @Copy Constructor
   **/
  OutputServer(const OutputServer&) = delete;
  /**
   * @Constructor
   **/
  OutputServer(const uint32_t agent_descovery_spin_rate,
               const uint32_t agent_callback_queue_length,
               const uint32_t agent_refresh_rate,
               const uint32_t publishers_queue_length,
               const uint32_t publishing_spin_rate);
  /**
   * @Deconstructor
   **/
  ~OutputServer();
private:
  /* Information needed by the publishers */
  const uint32_t m_publishers_queue_length;
  const uint32_t m_publishing_spin_rate;
  /* Holds all the agents currently present */
  AgentPool m_agent_pool;
  /* Holds all of the publishing objects */
  std::map<std::string, PositionPublisher> m_pubs;
  /* For ROS stuff */
  ros::NodeHandle c_nh;
  /* For making new topics */
  ros::ServiceServer make_topic_srv;
  /* For ending topics */
  ros::ServiceServer end_topic_srv;
  /**
   * @newSubscription
   *
   * @brief
   * Makes a new thread that will publish VehiclePoses in the frame that
   * is passed in.
   **/
  void newSubscription(rebroadcaste_msgs::ConnectPositionServer::Request  &req,
                       rebroadcaste_msgs::ConnectPositionServer::Response &res);
  /**
   * @endSubscription
   *
   * @brief
   * Closes a subscription.
   **/
  void endSubscription(rebroadcaste_msgs::DisconnectRebroadcast::Request  &req,
                       rebroadcaste_msgs::DisconnectRebroadcast::Response &res);
};

#endif
/* output_server.hpp */
