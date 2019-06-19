/**
 * @File: relay.hpp
 * @Date: 18 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Class should be used to relay messages from one topic to another.
 **/

/* Local Headers */
#include"topic_rebroadcaster/relay.hpp"

/* ROS Headers */
#include<ros/ros.h>
#include<ros/message_event.h>
#include<topic_tools/shape_shifter.h>

/* C++ Headers */
#include<string>
#include<thread>

Relay::Relay(const std::string& input_topic,
             const std::string& output_topic,
             const uint32_t     sub_queue_length,
             const uint32_t     pub_queue_length,
             const uint32_t     spin_rate)
 : m_input_topic(input_topic),
   m_output_topic(output_topic),
   c_nh(),
   m_pub(),
   m_sub(c_nh.subscribe(this->m_input_topic, sub_queue_length, &Relay::subCallback, this)),
   m_callback_queue(),
   m_pub_queue_length(pub_queue_length),
   pub_is_advertised(false),
   m_thread(&Relay::runRelay, std::ref(*this), spin_rate)
{
  this->c_nh.setCallbackQueue(&this->m_callback_queue); 
}

Relay::~Relay()
{
  this->m_callback_queue.disable();
  this->m_callback_queue.clear();
  this->m_thread.join();
  this->m_sub.shutdown();
  this->m_pub.shutdown();
}

const std::string& Relay::getInputTopic() const noexcept
{
  return this->m_input_topic; 
}

const std::string& Relay::getOutputTopic() const noexcept
{
  return this->m_output_topic;
}

void Relay::runRelay(const uint32_t spin_rate)
{
  ros::Rate loop_rate(spin_rate);

  while(this->c_nh.ok() && this->m_callback_queue.isEnabled())
  {
    this->m_callback_queue.callOne();
    loop_rate.sleep();
  }

  return;
}

void Relay::subCallback(const ros::MessageEvent<topic_tools::ShapeShifter>& msg_in)
{
  const boost::shared_ptr<const topic_tools::ShapeShifter> msg(msg_in.getConstMessage());

  if(!this->pub_is_advertised)
  {
    this->pub_is_advertised = true;
    this->m_pub = msg->advertise(this->c_nh, this->getOutputTopic(), this->m_pub_queue_length);
  }

  ros::Time timer = ros::Time::now();

  while((0 == this->m_pub.getNumSubscribers()) && (ros::Duration(1) > (ros::Time::now() - timer)));

  this->m_pub.publish(msg);
  return;
}

/* relay.cpp */

