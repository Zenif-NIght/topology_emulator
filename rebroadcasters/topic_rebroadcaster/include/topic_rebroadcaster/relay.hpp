/**
 * @File: relay.hpp
 * @Date: 18 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Class should be used to relay messages from one topic to another.
 **/

#ifndef RELAY_HPP
#define RELAY_HPP

/* ROS Headers */
#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<ros/message_event.h>
#include<topic_tools/shape_shifter.h>

/* C++ Headers */
#include<string>
#include<thread>

class Relay
{
public:
  /**
   * @Default Constructor
   **/
  Relay() = delete;
  /**
   * @Copy Constructor
   **/
  Relay(const Relay&) = delete;
  /**
   * @Constructor
   **/
  Relay(const std::string& input_topic,
        const std::string& output_topic,
        const uint32_t     sub_queue_length = 5,
        const uint32_t     pub_queue_length = 5,
        const uint32_t     spin_rate = 50);
  /**
   * @Deconstructor
   **/
  ~Relay();
  /**
   * @get
   **/
  const std::string& getInputTopic()  const noexcept;
  const std::string& getOutputTopic() const noexcept;
private:
  /* Holds identification information */
  const std::string m_input_topic;
  const std::string m_output_topic;
  /* Connects to topics */
  ros::NodeHandle c_nh;
  ros::Publisher  m_pub;
  ros::Subscriber m_sub;
  ros::CallbackQueue m_callback_queue;
  const uint32_t m_pub_queue_length;
  bool pub_is_advertised;
  /* Spins and does the relaying */
  std::thread m_thread;
  /**
   * @runRelay
   *
   * @brief
   * Function runs in thread to work the relay.
   **/
  void runRelay(const uint32_t spin_rate);
  /**
   * @subCallback
   *
   * @brief
   * Ran as a ROS Subscriber callback, publishes messages that come in.
   **/
  void subCallback(const ros::MessageEvent<topic_tools::ShapeShifter>& msg_in);
};

#endif
/* relay.hpp */

