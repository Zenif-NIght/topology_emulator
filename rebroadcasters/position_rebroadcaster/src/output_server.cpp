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
//#include<rebroadcaste_msgs/ConnectPositionServer.h>
//#include<rebroadcaste_msgs/DisconnectRebroadcast.h>

/* ROS Headers */
#include<ros/ros.h>

/* C++ Headers */
#include<map>
#include<string>

OutputServer::OutputServer(const uint32_t agent_descovery_spin_rate,
                           const uint32_t agent_callback_queue_length,
                           const uint32_t agent_refresh_rate,
                           const uint32_t publishers_queue_length,
                           const uint32_t publishing_spin_rate)
 : m_publishers_queue_length(publishers_queue_length),
   m_publishing_spin_rate(publishing_spin_rate),
   m_agent_pool(agent_descovery_spin_rate, agent_callback_queue_length, agent_refresh_rate),
   make_topic_srv(c_nh.advertiseService("position_rebroadcaster/connect",   &OutputServer::newSubscription, this)),
   end_topic_srv (c_nh.advertiseService("position_rebroadcaster/disconnect",&OutputServer::endSubscription, this)) 
{}

void OutputServer::newSubscription(rebroadcaste_msgs::ConnectPositionServer::Request  &req,
                                   rebroadcaste_msgs::ConnectPositionServer::Response &res)
{
  this->m_pubs.emplace(req.topic, req.topic, req.frameId, std::weak_ptr<AgentPool>(this->m_agent_pool),
                       this->m_publishers_queue_length, this->m_publishing_spin_rate);
  
  res.exitStatus = true;
  return;
}

void OutputServer::endSubscription(rebroadcaste_msgs::DisconnectRebroadcast::Request  &req,
                                   rebroadcaste_msgs::DisconnectRebroadcast::Response &res)
{
  this->m_pubs.erase(req.topic);

  res.exitStatus = true;
  return;
}


/* output_server.cpp */

