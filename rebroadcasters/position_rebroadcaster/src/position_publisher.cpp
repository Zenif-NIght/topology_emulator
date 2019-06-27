/**
 * @File: position_publisher.cpp
 * @Date: 16 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Handles transforming and sending VehiclePoses msgs to one client.
 **/

/* Local Headers */
#include"position_rebroadcasters/position_publisher.hpp"
#include"position_rebroadcasters/agent_pool.hpp"

/* mv_msgs Headers */
#include<mv_msgs/VehiclePoses.h>
#include<mv_msgs/VehiclePose.h>

/* network_topology_emulator Headers */
#include<network_topology_emulator/GetNeighbors.h>

/* ROS Headers */
#include<ros/ros.h>
#include<ros/time.h>
#include<tf/tf.h>
#include<tf/transform_listener.h>

#include<geometry_msgs/PoseStamped.h>

/* C++ Headers */
#include<string>
#include<memory>
#include<thread>
#include<stdexcept>
#include<functional>

/* Set to true if you want to see the output of this node in rvis */
#define VISUALIZE false

#if VISUALIZE
#include<nav_msgs/Odometry.h>
#endif

PositionPublisher::PositionPublisher(const std::string&                      output_topic,
                                     const std::string&                      output_frameId,
                                     const std::reference_wrapper<AgentPool> agents,
                                     const bool                              use_filter,
                                     const std::string&                      filter_topic,
                                     const uint32_t                          publisher_queue_length,
                                     const uint32_t                          publish_spin_rate)
 : m_use_filter(use_filter),
   m_frameId(output_frameId),
   m_agents(agents),
   m_pub(c_nh.advertise<mv_msgs::VehiclePoses>(output_topic, publisher_queue_length)),
   m_sub((use_filter) ? this->c_nh.serviceClient<network_topology_emulator::GetNeighbors>(filter_topic) : ros::ServiceClient()),
   run_thread(true),
   m_thread(&PositionPublisher::publishInThread, this, publish_spin_rate)
{}

PositionPublisher::~PositionPublisher()
{
  this->run_thread = false;
  this->m_thread.join();
  this->m_pub.shutdown();
}

const std::string& PositionPublisher::getFrameId() const noexcept
{
  return this->m_frameId;
}

void PositionPublisher::publishInThread(const uint32_t spin_rate)
{
  ros::Rate loop_rate(spin_rate);

  #if VISUALIZE
  ros::Publisher visualize_pub = this->c_nh.advertise<nav_msgs::Odometry>("visualize_transform_odom", 50);
  #endif

  while(this->c_nh.ok() && this->run_thread)
  {
    mv_msgs::VehiclePoses msg_out;

    // Will fail only if this node can't connect to the network_topology_emulator node
    if(this->getData(msg_out))
    {
      // Setup msg out
      msg_out.header.stamp    = ros::Time::now();
      msg_out.header.frame_id = this->getFrameId();

      this->transformData(msg_out);

      #if VISUALIZE
      for(uint32_t agent_it = 0; agent_it < msg_out.vehicles.size(); agent_it++)
      {
        nav_msgs::Odometry vis_msg;

        vis_msg.header    = msg_out.header;
        vis_msg.pose.pose = msg_out.vehicles.at(agent_it).pose.pose;

        visualize_pub.publish(vis_msg);
      }
      #endif

      this->m_pub.publish(msg_out);
    }
    loop_rate.sleep();
  }

  return;
}

bool PositionPublisher::getData(mv_msgs::VehiclePoses& poses) noexcept
{
  if(this->m_use_filter)
  {
    network_topology_emulator::GetNeighbors neighborhoodSet;

    // Get neighborhood set
    neighborhoodSet.request.robot_id = this->getFrameId();
    if(!this->m_sub.call(neighborhoodSet))
    {
      return false;
    }

    for(auto neighbor_it =  neighborhoodSet.response.neighbors.neighbors.cbegin();
             neighbor_it != neighborhoodSet.response.neighbors.neighbors.cend();
             neighbor_it++)
    {
      try
      {
        poses.vehicles.push_back(this->m_agents.get().getPose(*neighbor_it));
      }
      catch(const std::runtime_error& e)
      {} // If Agent isn't present in AgentPool
    }
  }
  else
  {
    poses = *this->m_agents.get().getAllPoses();
  }

  return true;
}

void PositionPublisher::transformData(mv_msgs::VehiclePoses& poses) const noexcept
{
  try
  {
    for(uint32_t agent_it = 0; agent_it < poses.vehicles.size(); agent_it++)
    {
      const geometry_msgs::PoseStamped pose_copy = poses.vehicles.at(agent_it).pose;

      // If transform is possible
      if(this->m_tfListener.waitForTransform(this->getFrameId(),
                                             pose_copy.header.frame_id,
                                             pose_copy.header.stamp,
                                             ros::Duration(0.5)))
      {
        this->m_tfListener.transformPose(this->getFrameId(),
                                         pose_copy,
                                         poses.vehicles.at(agent_it).pose);

        poses.vehicles.at(agent_it).pose.header.frame_id = this->getFrameId();
      }
      else
      {
        poses.vehicles.erase(poses.vehicles.begin() + agent_it);
      }
    }
  }
  catch(const tf::TransformException& e)
  {
    ROS_ERROR("PositionPublisher::publishInThread error, %s", e.what());
  }
}

/* position_publisher.cpp */

