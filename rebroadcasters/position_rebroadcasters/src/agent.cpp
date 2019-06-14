/**
 * @File: agent.cpp
 * @Date: 13 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * A class for describing an agents Pose in a multi vehicle framework.
 **/

/* Local Headers */
#include"position_rebroadcasters/agent.hpp"

/* Multi Vehicle Headers */
//#include"mv_msgs/VehiclePose.h"

/* C++ Headers */
#include<mutex>
#include<string>

/* ROS Headers */
#include<ros/ros.h>

#include<nav_msgs/Odometry.h>

Agent::Agent(const std::string& agent_name, const std::string& odom_topic, const uint32_t callbackQueueLegth)
 : m_name(agent_name)
{
  this->c_nh.setCallbackQueue(&this->m_callback_queue);

  this->m_odom_sub = c_nh.subscribe(odom_topic, callbackQueueLegth, &Agent::obomCallback, this);
}

Agent::~Agent()
{
  this->m_odom_sub.shutdown();
}

inline void Agent::update()
{
  std::unique_lock<std::mutex>(this->m_pose_mux);
  this->m_callback_queue.callAvailable();
}

inline const mv_msgs::VehiclePose& Agent::getPose() noexcept
{
  std::unique_lock<std::mutex>(this->m_pose_mux);
  return this->m_pose;
}

inline const mv_msgs::VehiclePose& Agent::updateGetPose()
{
  std::unique_lock<std::mutex>(this->m_pose_mux);
  this->m_callback_queue.callAvailable();
  return this->m_pose;
}

inline const std::string& Agent::getName() noexcept
{
  std::unique_lock<std::mutex>(this->m_pose_mux);
  return this->m_name;
}

/* agent.cpp */

