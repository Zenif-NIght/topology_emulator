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
   *
   * @brief
   * At the end of this constructor this object will be automatically looking
   * for new nav_msgs/Odometry messages to subscribe to.
   * @discovery_spin_rate: The frequency in hertz that this object will look for
   *                       robots to make Agent object for
   * @callback_queue_lengths: The size of the ROS callback queues that the Agent
   *                          objects will use
   * @agents_spin_rate: The frequency in hertz that the Agent objects will refresh
   *                    there pose data
   **/
  AgentPool(const uint32_t discovery_spin_rate,
            const uint32_t callback_queue_lengths = 5,
            const uint32_t agents_spin_rate = 50);
  /**
   * @Destructor
   **/
  ~AgentPool();
  /**
   * @getPose
   *
   * @brief
   * Returns the pose that is held by the passed in agent in a thread
   * safe way.
   * @agent_name: The ROS namespace that the robot publishes on
   **/
  mv_msgs::VehiclePose getPose(const std::string& agent_name);
  /**
   * @getAllPoses
   *
   * @brief
   * Returns all the pose data held by the pool at this time in a thread
   * safe way. Note, doesn't fill out the mv_msgs/VehiclePoses header.
   **/
  std::shared_ptr<mv_msgs::VehiclePoses> getAllPoses();
  /**
   * @getAgent
   *
   * @brief
   * Returns a pointer to the agent who's name you passed in. If agent
   * doesn't exist it will return nullptr.
   * @agent_name: The ROS namespace that the robot publishes on
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
  /* When false the thread will exit */
  bool m_thread_run;
  /* Keeps this object up to date */
  std::thread m_thread;
  /**
   * @updateThreadFunction
   *
   * @brief
   * To be ran in a thread that will check for new robots to add and
   * make sure old robots are still present.
   * @spin_rate: The frequency in hertz that this object will look for
   *             robots to make Agent object for
   **/
  void updateThreadFunction(const uint32_t spin_rate);
  /**
   * @currentAgents
   *
   * @brief
   * Returns a list of all the current Agents.
   **/
  std::shared_ptr<std::list<std::reference_wrapper<const std::string>>> currentAgents();
  /**
   * @getNamespace
   *
   * @brief
   * Fills the passed in string with the bots namespace if the passed in topic
   * is a odom topic.
   * @the_namespace: A empty string when its passed in and is filled with the robots
   *                 publishing namespace
   * @topic: The topic that you want to investigate
   * @return: True if it is an odom topic and false if not
   **/
  bool getNamespace(std::string& the_namespace, const std::string& topic);
  /**
   * @checkNamespace
   *
   * @brief
   * Checks if the list holds the passed in string.
   * @return: If it finds it, it will remove that element and return true
   *          If is doesn't it returns false
   **/
  bool checkNamespace(const std::string& the_namespace, std::list<std::reference_wrapper<const std::string>>& agents_list);
};

#endif
/* agent_pool.hpp */
