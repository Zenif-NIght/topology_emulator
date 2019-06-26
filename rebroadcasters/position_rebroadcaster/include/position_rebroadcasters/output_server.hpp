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

/* rebroadcaster_msgs Headers */
#include<rebroadcaster_msgs/ConnectPositionServer.h>
#include<rebroadcaster_msgs/DisconnectRebroadcast.h>

/* ROS Headers */
#include<ros/ros.h>

/* C++ Headers */
#include<map>
#include<string>
#include<mutex>

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
   *
   * @brief
   * At the end of creation this object will be ready to
   * receive connect and disconnect service calls.
   * @network_topology_topic: The topic that this object can use to get neighborhood
   *                          set information
   * @agent_discovery_spin_rate: The frequency in hertz that the AgentPool object
   *                             will look for new robots and remove old ones
   * @agent_callback_queue_length: The length of the callback queue each Agent object
   *                               will receive on
   * @agent_refresh_rate: The frequency in hertz that each Agent object will update its
   *                      pose data
   * @publishers_queue_length: The size of each PositionPublisher's ROS queue
   * @publishing_spin_rate: The frequency in hertz that each PositionPublisher object
   *                        will publish pose data
   **/
  OutputServer(const std::string& network_topology_topic,
               const uint32_t     agent_discovery_spin_rate,
               const uint32_t     agent_callback_queue_length,
               const uint32_t     agent_refresh_rate,
               const uint32_t     publishers_queue_length,
               const uint32_t     publishing_spin_rate);
  /**
   * @Deconstructor
   **/
  ~OutputServer() = default;
private:
  /* Information needed by the publishers */
  const std::string m_network_topology_topic;
  const uint32_t    m_publishers_queue_length;
  const uint32_t    m_publishing_spin_rate;
  /* Protects the add and remove operations of the map */
  std::mutex m_mux;
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
  bool newSubscription(rebroadcaster_msgs::ConnectPositionServer::Request  &req,
                       rebroadcaster_msgs::ConnectPositionServer::Response &res);
  /**
   * @endSubscription
   *
   * @brief
   * Closes a subscription.
   **/
  bool endSubscription(rebroadcaster_msgs::DisconnectRebroadcast::Request  &req,
                       rebroadcaster_msgs::DisconnectRebroadcast::Response &res);
};

#endif
/* output_server.hpp */
