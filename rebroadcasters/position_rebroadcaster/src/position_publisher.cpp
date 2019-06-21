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

PositionPublisher::PositionPublisher(const std::string&             outputTopic,
                                     const std::string&             outputFrameId,
                                     const std::weak_ptr<AgentPool> agents,
                                     const uint32_t                 publisher_queue_length,
                                     const uint32_t                 publish_spin_rate)
 : m_frameId(outputFrameId),
   m_topic(outputTopic),
   m_agents(agents),
   publish(true),
   m_thread(&PositionPublisher::publishInThread, std::ref(*this), publisher_queue_length, publish_spin_rate)
{}

PositionPublisher::~PositionPublisher()
{
  this->publish = false;
  this->m_thread.join();
}

const std::string& PositionPublisher::getFrameId() const noexcept
{
  return this->m_frameId; 
}

const std::string& PositionPublisher::getTopic() const noexcept
{
  return this->m_topic;
}

void PositionPublisher::publishInThread(const uint32_t queue_length, const uint32_t spin_rate)
{
  ros::NodeHandle t_nh;
  ros::Publisher t_pub = t_nh.advertise<mv_msgs::VehiclePoses>(this->getTopic(), queue_length);
  ros::Rate loop_rate(spin_rate);

  while(t_nh.ok() && this->publish)
  {
    // Get data
    std::shared_ptr<mv_msgs::VehiclePoses> raw_poses(this->m_agents.lock()->getAllPoses());
    
    // Setup msg out
    mv_msgs::VehiclePoses msg_out;
    msg_out.header.stamp = ros::Time::now();
    msg_out.header.frame_id = this->getFrameId();
    msg_out.vehicles.resize(raw_poses->vehicles.size());

    // Transform data
    try
    {
      for(uint32_t agent_it = 0; agent_it < raw_poses->vehicles.size(); agent_it++)
      {
        const geometry_msgs::PoseStamped& pose_ref = raw_poses->vehicles.at(agent_it).pose;

        if(this->m_tfListener.waitForTransform(this->m_frameId,
                                               pose_ref.header.frame_id,
                                               pose_ref.header.stamp,
                                               ros::Duration(0.5)))
        {
          this->m_tfListener.transformPose(this->getFrameId(),
                                           pose_ref,
                                           msg_out.vehicles.at(agent_it).pose);
        }
      }
    }
    catch(const tf::TransformException& e)
    {
      ROS_ERROR("PositionPublisher::publishInThread error, %s", e.what());
    }

    t_pub.publish(msg_out);
    loop_rate.sleep();
  }

  return;
}

/* position_publisher.cpp */

