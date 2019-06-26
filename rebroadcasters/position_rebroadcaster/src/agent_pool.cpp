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
#include"mv_msgs/VehiclePose.h"
#include"mv_msgs/VehiclePoses.h"

/* ROS Headers */
#include<ros/ros.h>

/* C++ Headers */
#include<string>
#include<mutex>
#include<list>
#include<memory>
#include<functional>
#include<stdexcept>

AgentPool::AgentPool(const uint32_t discovery_spin_rate,
                     const uint32_t callback_queue_lengths,
                     const uint32_t agents_spin_rate)
 : m_callback_queue_legths(callback_queue_lengths),
   m_agent_spin_rate(agents_spin_rate),
   m_thread_run(true),
   m_thread(&AgentPool::updateThreadFunction, std::ref(*this), discovery_spin_rate)
{}

AgentPool::~AgentPool()
{
  this->m_thread_run = false;
  this->m_thread.join();
  this->m_agents.clear();
}

mv_msgs::VehiclePose AgentPool::getPose(const std::string& agent_name)
{
  std::unique_lock<std::mutex> func_lock(this->m_mux);
  
  for(auto agent_it = this->m_agents.cbegin(); agent_it != this->m_agents.cend(); agent_it++)
  {
    if(agent_name == (*agent_it)->getName())
    {
      std::unique_lock<std::mutex> agent_lock((*agent_it)->getLock());
      return (*agent_it)->getPose();
    }
  }

  throw std::runtime_error("AgentPool::getPose error, Agent not present in pool");
}

std::shared_ptr<mv_msgs::VehiclePoses> AgentPool::getAllPoses()
{
  std::shared_ptr<mv_msgs::VehiclePoses> out_msg(new mv_msgs::VehiclePoses());
  std::unique_lock<std::mutex> func_lock(this->m_mux);

  for(auto agent_it = this->m_agents.cbegin(); agent_it != this->m_agents.cend(); agent_it++)
  {
    (*agent_it)->getLock().lock();
    
    const mv_msgs::VehiclePose& pose_ref = (*agent_it)->getPose();

    // If agent is fully initialized
    if(std::string() != pose_ref.pose.header.frame_id)
    {
      out_msg->vehicles.push_back(pose_ref);
    }
    (*agent_it)->getLock().unlock();
  }

  return out_msg;
}

std::shared_ptr<Agent> AgentPool::getAgent(const std::string& agent_name)
{
  std::unique_lock<std::mutex> func_lock(this->m_mux);
  
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
    // Get all open ROS topic
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    this->m_mux.lock();

    // Get all of the agent names this object currently has
    std::shared_ptr<std::list<std::reference_wrapper<const std::string>>> agents_list(this->currentAgents());

    // For all ROS topics
    for(auto topic_it = master_topics.cbegin(); topic_it != master_topics.cend(); topic_it++)
    {
      std::string the_namespace = std::string();

      // If the topic is a odom topic
      if(this->getNamespace(the_namespace, topic_it->name))
      {
        // If we don't already have this agent
        if(!this->checkNamespace(the_namespace, *agents_list))
        {
          // Make an agent for that namespace
          this->m_agents.emplace_back(new Agent(the_namespace, topic_it->name, this->m_callback_queue_legths, this->m_agent_spin_rate));
        }
      }
    }
    
    // If this object has any agents that don't have odom topics anymore
    if(!agents_list->empty())
    {  
      // Remove those agents
      this->m_agents.remove_if([&agents_list](const std::shared_ptr<Agent> agent_it) -> bool
        {
          for(auto list_it = agents_list->cbegin(); list_it != agents_list->cend(); list_it++)
          {
            if(list_it->get() == agent_it->getName())
            {
              agents_list->erase(list_it);
              return true;
            }
          }
          return false;
        });
    }

    this->m_mux.unlock();
    loop_rate.sleep();
  }

  return;
}

std::shared_ptr<std::list<std::reference_wrapper<const std::string>>> AgentPool::currentAgents()
{
  std::shared_ptr<std::list<std::reference_wrapper<const std::string>>> list(new std::list<std::reference_wrapper<const std::string>>());

  for(auto agent_it = this->m_agents.cbegin(); agent_it != this->m_agents.cend(); agent_it++)
  {
    list->emplace_back(std::ref((*agent_it)->getName()));
  }

  return list;
}

bool AgentPool::getNamespace(std::string& the_namespace, const std::string& topic)
{
  uint32_t slash_pos = topic.find_last_of("/");

  if(std::string("odom") != topic.substr(slash_pos + 1, topic.size() - 1))
  {
    return false;
  }
  
  the_namespace = topic.substr(1, topic.size() - slash_pos + 1);
  return true; 
}

bool AgentPool::checkNamespace(const std::string& the_namespace, std::list<std::reference_wrapper<const std::string>>& agents_list)
{
  for(auto list_it = agents_list.cbegin(); list_it != agents_list.cend(); list_it++)
  {
    // We have this agent
    if(the_namespace == list_it->get())
    {
      agents_list.erase(list_it);
      return true;
    }
  }
  return false;
}

/* agent_pool.cpp */

