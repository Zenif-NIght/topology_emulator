/**
 * @File: relay_server.hpp
 * @Date: 18 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Class to handle the creation and destruction of Relay objects.
 **/

#ifndef RELAY_SERVER_HPP
#define RELAY_SERVER_HPP

/* Local Headers */
#include"topic_rebroadcaster/relay.hpp"

/* Rebroadcaster_msgs Headers */
#include<rebroadcaster_msgs/ConnectTopics.h>
#include<rebroadcaster_msgs/DisconnectRebroadcast.h>

/* ROS Headers */
#include<ros/ros.h>

/* C++ Headers */
#include<string>
#include<map>
#include<mutex>

class RelayServer
{
public:
  /**
   * @Default Constructor
   *
   * @brief
   * After initialization the object will automatically handle ROS service
   * calls and constructing and deconstructing Relay objects.
   **/
  RelayServer();
  /**
   * @Copy Constructor
   **/
  RelayServer(const RelayServer&) = delete;
  /**
   * @Deconstructor
   **/
  ~RelayServer() = default;
private:
  /* Holds the Relay objects */
  std::map<std::string, Relay> m_relays;
  /* For making new and removing Relays */
  ros::NodeHandle c_nh;
  ros::ServiceServer add_srv;
  ros::ServiceServer remove_srv;
  /* Protects insertions and removals */
  std::mutex m_mux;
  /**
   * @addCallback
   *
   * @brief
   * Adds a new Relay with the passed in parameters.
   **/
  bool addCallback(rebroadcaster_msgs::ConnectTopics::Request&  req,
                   rebroadcaster_msgs::ConnectTopics::Response& res);
  /**
   * @removeCallback
   *
   * @brief
   * Removes the Relay with the passed in parameters.
   **/
  bool removeCallback(rebroadcaster_msgs::DisconnectRebroadcast::Request&  req,
                      rebroadcaster_msgs::DisconnectRebroadcast::Response& res);
};

#endif
/* relay_server.hpp */

