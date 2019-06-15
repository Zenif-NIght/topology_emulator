/**
 * @File: agent_pool.hpp
 * @Date: 14 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Handles keeping all agents updated and thread safe.
 **/

#ifndef AGENT_POOL_HPP
#define AGENT_POOL_HPP

/* Local Headers */
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
#include<vector>

class AgentPool 
{
public:
  /**
   * @Default Constructor
   **/
  AgentPool() = delete;
  /**
   * @Copy Constructor
   **/
  AgentPool(const AgentPool&) = delete;
  /**
   * @Constructor
   **/
  AgentPool(const uint32_t descovery_spin_rate = 1,
            const uint32_t callback_queue_legths = 5,
            const uint32_t agents_spin_rate = 50);
  /**
   * @Destructor
   **/
  ~AgentPool();
  /**
   * @getPose
   *
   * @brief
   * Returns the pose that is held by the passed in agent.
   **/
  const mv_msgs::VehiclePose& getPose(const std::string& agent_name);
  /**
   * @getAllPoses
   *
   * @brief
   * Returns all the pose data held by the pool at this time.
   * Note, doesn't fill out header.
   **/
  const mv_msgs::VehiclePoses& getAllPoses() noexcept;
  /**
   * @getAgent
   *
   * @brief
   * Returns a pointer to the agent who's name you passed in.
   **/
  std::shared_ptr<Agent> getAgent(const std::string& agent_name);
private:
  /* Holds Agents */
  std::list<std::shared_ptr<Agent>> m_agents;
  /* Data held for Agent creation */
  const uint32_t m_callback_queue_legths;
  const uint32_t m_agent_spin_rate; 
  /* Prevents race conditions */
  std::mutex m_mux;
  /* Keeps this object up to date */
  std::thread m_thread;
  /* When false the thread will exit */
  bool m_thread_run;
  /* Checks to see if this is the odom topic */
  const std::regex odom_regex;
  /**
   * @updateThreadFunction
   *
   * @brief
   * To be ran in a thread that will check for new robots to add and
   * make sure old robots are still present.
   **/
  void updateThreadFunction(const uint32_t spin_rate);
  /**
   * @currentAgents
   *
   * @brief
   * Returns a list of all the current Agents.
   **/
  std::shared_ptr<std::list<std::string&>> currentAgents();
  /**
   * @getNamespace
   *
   * @brief
   * Fills the passed in string with the bots namespace if the passed in topic
   * is a odom topic.
   **/
  bool getNamespace(std::string& the_namespace, const std::string& topic);
  /**
   * @checkNamespace
   *
   * @brief
   * Checks in the list for the passed in string.
   * @return: If it finds it, it will remove that element and return true
   *          If is doesn't it returns false
   **/
  bool checkNamespace(const std::string& the_namespace, std::list<std::string&>& agents_list);
};

#endif
/* agent_pool.hpp */
