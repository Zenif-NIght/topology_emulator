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
#include"mv_msgs/VehiclePose.h"

/* C++ Headers */
#include<mutex>
#include<string>

/* ROS Headers */
#include<ros/ros.h>

#include<nav_msgs/Odometry.h>

Agent::Agent(const std::string& agent_name,
             const std::string& odom_topic,
             const uint32_t callback_queue_legth,
             const uint32_t spin_rate)
 : m_name(agent_name),
   m_thread(&Agent::threadFunction, std::ref(*this), spin_rate)
{
  this->c_nh.setCallbackQueue(&this->m_callback_queue);

  this->m_odom_sub = c_nh.subscribe(odom_topic, callback_queue_legth, &Agent::odomCallback, this);
}

Agent::~Agent()
{
  this->getLock().lock();

  this->m_callback_queue.clear();
  this->m_callback_queue.disable();
  this->m_odom_sub.shutdown();

  this->getLock().unlock();

  this->m_thread.join();
}

const mv_msgs::VehiclePose& Agent::getPose() noexcept
{
  std::unique_lock<std::mutex>(this->getLock());
  return this->m_pose;
}

const std::string& Agent::getName() noexcept
{
  std::unique_lock<std::mutex>(this->getLock());
  return this->m_name;
}

const std::string& Agent::getFrameId() noexcept
{
  std::unique_lock<std::mutex>(this->getLock());
  return this->m_pose.pose.header.frame_id;
}

std::mutex& Agent::getLock() noexcept
{
  return this->m_pose_mux;
}

void Agent::odomCallback(const nav_msgs::Odometry& msg_in)
{
  std::unique_lock<std::mutex>(this->getLock());

  this->m_pose.robot_id = this->m_name;
  this->m_pose.pose.header = msg_in.header;
  this->m_pose.pose.pose = msg_in.pose.pose;
}

void Agent::threadFunction(const uint32_t spin_rate)
{
  ros::Rate loop_rate(spin_rate);

  while(this->c_nh.ok() && this->m_callback_queue.isEnabled())
  {
    this->m_callback_queue.callAvailable();
    loop_rate.sleep();
  }

  return;
}

/* agent.cpp */

