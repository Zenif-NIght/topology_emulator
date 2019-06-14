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

/* ROS Headers */
#include<ros/ros.h>

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
  Agent(const std::string& agent_name, const std::string& odom_topic); 
  /**
   * @Deconstructor
   **/
  ~Agent();
  /**
   * @update
   *
   * @brief
   * Updates this objects pose if there is a callback to run
   * in the ROS callback queue.
   **/
  void update();
  /**
   * @
private:
  /* This agents unique identifier */
  std::string m_name;
  /* For initializing this class */
  ros::NodeHandle c_nh;
  /* For updating Pose data */
  ros::Subscriber m_odom_sub;
  /* Holds the latest data from odom */
  mv_msgs::VehiclePose m_pose;
  /* Protects this object from race conditions */
  std::mutex m_pose_mux;
  /* Gets information from odom */
  void odomCallback(const nav_msgs::Odometry& msg_in);
};

#endif
/* agent.hpp */
