/**
 * @File: output_server.hpp
 * @Date: 16 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * This class is made to oversee the creation and destruction of vehicle position
 * message topics.
 **/

/* Local Headers */
#include"position_rebroadcasters/output_server.hpp"
#include"position_rebroadcasters/agent_pool.hpp"
#include"position_rebroadcasters/position_publisher.hpp"

/* rebroadcaste_msgs Headers */
#include<rebroadcaster_msgs/ConnectPositionServer.h>
#include<rebroadcaster_msgs/DisconnectRebroadcast.h>

/* ROS Headers */
#include<ros/ros.h>

/* C++ Headers */
#include<map>
#include<string>

OutputServer::OutputServer(const uint32_t agent_discovery_spin_rate,
                           const uint32_t agent_callback_queue_length,
                           const uint32_t agent_refresh_rate,
                           const uint32_t publishers_queue_length,
                           const uint32_t publishing_spin_rate)
 : m_publishers_queue_length(publishers_queue_length),
   m_publishing_spin_rate(publishing_spin_rate),
   m_agent_pool(agent_discovery_spin_rate, agent_callback_queue_length, agent_refresh_rate),
   make_topic_srv(c_nh.advertiseService("position_rebroadcaster/connect",   &OutputServer::newSubscription, this)),
   end_topic_srv (c_nh.advertiseService("position_rebroadcaster/disconnect",&OutputServer::endSubscription, this)) 
{}

bool OutputServer::newSubscription(rebroadcaster_msgs::ConnectPositionServer::Request  &req,
                                   rebroadcaster_msgs::ConnectPositionServer::Response &res)
{
  this->m_pubs.emplace(std::piecewise_construct,
                       std::forward_as_tuple(std::string(req.topic)),
                       std::forward_as_tuple(
                         std::string(req.topic),
                         std::string(req.frameId),
                         std::weak_ptr<AgentPool>(std::shared_ptr<AgentPool>(&this->m_agent_pool)),
                         this->m_publishers_queue_length,
                         this->m_publishing_spin_rate));
  
  return true;
}

bool OutputServer::endSubscription(rebroadcaster_msgs::DisconnectRebroadcast::Request  &req,
                                   rebroadcaster_msgs::DisconnectRebroadcast::Response &res)
{
  this->m_pubs.erase(req.topic);

  return true;
}


/* output_server.cpp */

