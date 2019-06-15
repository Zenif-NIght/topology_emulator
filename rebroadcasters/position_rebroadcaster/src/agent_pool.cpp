/**
 * @File: agent_pool.cpp
 * @Date: 14 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Handles keeping all agents updated and thread safe.
 **/

/* Local Headers */
#include"position_rebroadcasters/agent_pool.hpp"
#include"position_rebroadcasters/agent.hpp"

/* Multi Vehicle Headers */
// #include"mv_msgs/VehiclePose.h"
// #include"mv_msgs/VehiclePoses.h"

/* ROS Headers */
#include<ros/ros.h>

/* C++ Headers */
#include<string>
#include<mutex>
#include<list>
#include<memory>
#include<regex>

AgentPool::AgentPool(const uint32_t descovery_spin_rate,
                     const uint32_t callback_queue_legths,
                     const uint32_t agents_spin_rate)
 : m_callback_queue_legths(callback_queue_legths),
   m_agent_spin_rate(agents_spin_rate),
   m_thread(&AgentPool::updateThreadFunction, std::ref(*this), descovery_spin_rate),
   m_thread_run(true),
   odom_regex("(.*)(/odem)($)", std::regex::basic)
{}

AgentPool::~AgentPool()
{
  this->m_thread_run = false;
  this->m_thread.join();
  this->m_agents.clear();
}

const mv_msgs::VehiclePose& AgentPool::getPose(const std::string& agent_name)
{
  std::unique_lock<std::mutex>(this->m_mux);
  
  for(auto agent_it = this->m_agents.cbegin(); agent_it != this->m_agents.cend(); agent_it++)
  {
    if(agent_name == (*agent_it)->getName())
    {
      return (*agent_it)->getPose();
    }
  }

  throw std::runtime_error("AgentPool::getPose error, Agent not present in pool");
}

const mv_msgs::VehiclePoses& AgentPool::getAllPoses() noexcept
{
  std::unique_lock<std::mutex>(this->m_mux);
  mv_msgs::VehiclePoses out_msg;

  out_msg.vehicles.resize(this->m_agents.size());

  for(auto agent_it = this->m_agents.cbegin(); agent_it != this->m_agents.cend(); agent_it++)
  {
    out_msg.vehicles.push_back((*agent_it)->getPose());
  }

  return out_msg;
}

std::shared_ptr<Agent> AgentPool::getAgent(const std::string& agent_name)
{
  std::unique_lock<std::mutex>(this->m_mux);
  
  for(auto agent_it = this->m_agents.cbegin(); agent_it != this->m_agents.cend(); agent_it++)
  {
    if(agent_name == (*agent_it)->getName())
    {
      return *agent_it;
    }
  }

  throw std::runtime_error("AgentPool::getAgent error, Agent not present in pool");
}

void AgentPool::updateThreadFunction(const uint32_t spin_rate)
{
  ros::Rate loop_rate(spin_rate);

  while(ros::ok() && this->m_thread_run)
  {
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    this->m_mux.lock();

    std::shared_ptr<std::list<std::string&>> agents_list(this->currentAgents());

    // For all ROS topics
    for(auto topic_it = master_topics.cbegin(); topic_it != master_topics.cend(); topic_it++)
    {
      std::string the_namespace = std::string();

      // If the topic is a odom topic
      if(this->getNamespace(the_namespace, topic_it->name))
      {
        // If we don't already have this agent
        if(!this->checkNamespace(the_namespace, *agents_list.get()))
        {
          // Make an agent for that namespace
          this->m_agents.emplace_back(the_namespace, topic_it->name, this->m_callback_queue_legths, this->m_agent_spin_rate);
        }
      }
    }
    
    // If this object has any agents that don't have odom topics anymore
    if(!agents_list->empty())
    {
      // Remove those agents
      for(auto current_it = this->m_agents.begin(); current_it != this->m_agents.end(); current_it++)
      {
        for(auto remove_it = agents_list->cbegin(); remove_it != agents_list->cend(); remove_it++)
        {
          if(*remove_it == (*current_it)->getName())
          {
            this->m_agents.erase(current_it);
            agents_list->erase(remove_it);
            break;
          }
        }
      }
    }

    this->m_mux.unlock();
    loop_rate.sleep();
  }

  return;
}

std::shared_ptr<std::list<std::string&>> AgentPool::currentAgents()
{
  std::shared_ptr<std::list<std::string&>> list(new std::list<std::string&>());

  for(auto agent_it = this->m_agents.cbegin(); agent_it != this->m_agents.cend(); agent_it++)
  {
    list->emplace_back((*agent_it)->getName());
  }

  return list;
}

bool AgentPool::getNamespace(std::string& the_namespace, const std::string& topic)
{
  // See if it matches (.*)(/odem)($)
  if(!std::regex_match(topic, this->odom_regex))
  {
    return false;
  }

  uint32_t slash_pos = topic.find_last_of("/");
  the_namespace = topic.substr(1, topic.size() - slash_pos);
  return true; 
}

bool AgentPool::checkNamespace(const std::string& the_namespace, std::list<std::string&>& agents_list)
{
  for(auto list_it = agents_list.begin(); list_it != agents_list.end(); list_it++)
  {
    // We have this agent
    if(the_namespace == *list_it)
    {
      agents_list.erase(list_it);
      return true;
    }
  }
  return false;
}

/* agent_pool.cpp */

