/**
 * @File: agent.hpp
 * @Date: 13 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * A class for describing an agents Pose in a multi vehicle framework.
 **/

#ifndef AGENT_HPP
#define AGENT_HPP

/* Multi Vehicle Headers */
//#include"mv_msgs/VehiclePose.h"

/* C++ Headers */
#include<mutex>
#include<string>
#include<thread>

/* ROS Headers */
#include<ros/ros.h>
#include<ros/callback_queue.h>

#include<nav_msgs/Odometry.h>

class Agent
{
public:
  /**
   * @Default Constructor
   **/
  Agent() = delete;
  /**
   * @Copy Constructor
   **/
  Agent(const Agent&) = delete;
  /**
   * @Constructor
   **/
  Agent(const std::string& agent_name,
        const std::string& odom_topic,
        const uint32_t callback_queue_legth = 5,
        const uint32_t spinRate = 50);
  /**
   * @Deconstructor
   **/
  ~Agent();
  /**
   * @getPose
   *
   * @brief
   * Returns the objects pose data.
   **/
  const mv_msgs::VehiclePose& getPose() noexcept;
  /**
   * @getName
   *
   * @brief
   * Returns this objects name.
   **/
  const std::string& getName() noexcept;
  /**
   * @getFrameId
   *
   * @brief
   * Returns this objects frame id.
   **/
  const std::string& getFrameId() noexcept;
  /**
   * @getLock
   *
   * @brief
   * Returns a reference to this objects mutex.
   **/
  std::mutex& getLock() noexcept;
private:
  /* This agents unique identifier */
  std::string m_name;
  /* For initializing this class */
  ros::NodeHandle c_nh;
  /* For holding callback that haven't been ran yet */
  ros::CallbackQueue m_callback_queue;
  /* For updating Pose data */
  ros::Subscriber m_odom_sub;
  /* Holds the latest data from odom */
  mv_msgs::VehiclePose m_pose;
  /* Protects this object from race conditions */
  std::mutex m_pose_mux;
  /* Keeps this object up to date */
  std::thread m_thread;
  /**
   * @threadFunction
   *
   * @brief
   * Runs odomCallbacks as they come.
   **/
  void threadFunction(const uint32_t spin_rate);
  /**
   * @odomCallback
   *
   * @brief
   * Gets information from odom.
   **/
  void odomCallback(const nav_msgs::Odometry& msg_in);
};

#endif
/* agent.hpp */
