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
#include<memory>
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
   **/
  PositionPublisher(const std::string&                      outputTopic,
                    const std::string&                      outputFrameId,
                    const std::reference_wrapper<AgentPool> agents,
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
   * Returns the value asked for.
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
  /* For getting the neiborhood set */
  ros::ServiceClient m_sub;
  /* Tells the thread when to shutdown */
  bool run_thread;
  /* The thread that will do the publishing */
  std::thread m_thread;
  /**
   * @publishInThread
   *
   * @brief
   * When ran in a thread object this function will translate and publish msgs
   * at the passed in speed.
   **/
  void publishInThread(const uint32_t spin_rate);
  /**
   * @getData
   *
   * @brief
   * Fills the passed in object with the vehicle poses that need to be
   * processed.
   * @return: True an success and false on failure
   **/
  bool getData(mv_msgs::VehiclePoses& poses) noexcept;
  /**
   * @tranformData
   *
   * @brief
   * Transforms all of the passed in messages' poses to this objects frame.
   **/
  void transformData(mv_msgs::VehiclePoses& poses) const noexcept;
};

#endif
/* position_publisher.hpp */
