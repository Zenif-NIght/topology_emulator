/**
 * @File: position_publisher.hpp
 * @Date: 16 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Handles transforming and sending VehiclePoses msgs to one client.
 **/

#ifndef POSITION_PUBLISHER_HPP
#define POSITION_PUBLISHER_HPP

/* Local Headers */
#include"position_rebroadcasters/agent_pool.hpp"

/* mv_msgs Headers */
#include<mv_msgs/VehiclePoses.h>

/* ROS Headers */
#include<ros/ros.h>
#include<tf/tf.h>
#include<tf/transform_listener.h>

/* C++ Headers */
#include<string>
#include<thread>
#include<functional>

class PositionPublisher
{
public:
  /**
   * @Default Constructor
   **/
  PositionPublisher() = delete;
  /**
   * @Copy Constructor
   **/
  PositionPublisher(const PositionPublisher&) = delete;
  /**
   * @Constructor
   *
   * @brief
   * At the end of construction this object will automatically retrieve needed
   * pose data, transform it to the correct frame id, and publish it at the
   * given frequency.
   * @output_topic: The topic this object will publish on
   * @output_frameId: The frame id that all the poses will be in when they are published
   * @agents: A reference to a shared AgentPool
   * @use_filter: Whether or not to only publish this frameId's neighborhood set
   * @filter_topic: The topic that this object will use to make service calls to the
   *                network_topology_emulator node to get neighborhood sets
   * @publisher_queue_length: The size of this object ROS queue length
   * @publish_spin_rate: The frequency in hertz that this object will publish
   *                     mv_msgs/VehiclePoses messages
   **/
  PositionPublisher(const std::string&                      output_topic,
                    const std::string&                      output_frameId,
                    const std::reference_wrapper<AgentPool>&agents,
                    const bool                              use_filter,
                    const std::string&                      filter_topic,
                    const uint32_t                          publisher_queue_length = 5,
                    const uint32_t                          publish_spin_rate = 50);
  /**
   * @Deconstructor
   **/
  ~PositionPublisher();
  /**
   * @get
   *
   * @brief
   * Returns this objects frame id.
   **/
  const std::string& getFrameId() const noexcept;
private:
  /* Whether or not to only publish the neighborhood set */
  const bool m_use_filter;
  /* The frame that this class publishes data in */
  const std::string m_frameId;
  /* Used to transform frames */
  tf::TransformListener m_tfListener;
  /* Holds the position data */
  const std::reference_wrapper<AgentPool> m_agents;
  /* ROS stuff */
  ros::NodeHandle c_nh;
  /* For publishing poses */
  ros::Publisher m_pub;
  /* For getting the neighborhood set */
  ros::ServiceClient m_sub;
  /* Tells the thread when to shutdown */
  bool run_thread;
  /* The thread that will do the publishing */
  std::thread m_thread;
  /**
   * @publishInThread
   *
   * @brief
   * When ran in a thread object this function will translate and publish msgs.
   * @spin_rate: The frequency in hertz that this function will publish
   *             messages
   **/
  void publishInThread(const uint32_t spin_rate);
  /**
   * @getData
   *
   * @brief
   * Fills the passed in object with the vehicle poses that need to be
   * processed.
   * @poses: An empty object when it's passed in and is filled with pose
   *         data on success
   * @return: True an success and false on failure
   **/
  bool getData(mv_msgs::VehiclePoses& poses) noexcept;
  /**
   * @tranformData
   *
   * @brief
   * Transforms all of the passed in messages' poses to this objects frame.
   * @poses: An object holding pose data
   **/
  void transformData(mv_msgs::VehiclePoses& poses) const noexcept;
};

#endif
/* position_publisher.hpp */
