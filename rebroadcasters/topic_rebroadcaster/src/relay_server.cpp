/**
 * @File: relay_server.cpp
 * @Date: 18 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Class to handle the creation and destruction of Relay objects.
 **/

/* Local Headers */
#include"topic_rebroadcaster/relay_server.hpp"
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
#include<functional>

RelayServer::RelayServer()
 : add_srv   (c_nh.advertiseService("topic_rebroadcaster/connect",    &RelayServer::addCallback,    this)),
   remove_srv(c_nh.advertiseService("topic_rebroadcaster/disconnect", &RelayServer::removeCallback, this))
{}

bool RelayServer::addCallback(rebroadcaster_msgs::ConnectTopics::Request&  req,
                              rebroadcaster_msgs::ConnectTopics::Response& res)
{
  std::unique_lock<std::mutex>(this->m_mux);

  this->m_relays.emplace(std::piecewise_construct,
                         std::forward_as_tuple(req.topic_out),
                         std::forward_as_tuple(
                           req.topic_in,
                           req.topic_out,
                           (0 == req.queue_length) ? 5 : req.queue_length,
                           (0 == req.queue_length) ? 5 : req.queue_length,
                           (0 == req.spin_rate) ? 30 : req.spin_rate));
  return true;
}

bool RelayServer::removeCallback(rebroadcaster_msgs::DisconnectRebroadcast::Request&  req,
                                 rebroadcaster_msgs::DisconnectRebroadcast::Response& res)
{
  std::unique_lock<std::mutex>(this->m_mux);

  this->m_relays.erase(req.topic);

  return true;
}

/* relay_server.cpp */

