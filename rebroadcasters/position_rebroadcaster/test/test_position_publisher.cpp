/**
 * @File: test_position_publisher.cpp
 * @Date: 16 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the PositionPublisher object.
 **/

/* Google Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include"position_rebroadcasters/position_publisher.hpp"
#include"position_rebroadcasters/agent_pool.hpp"

/* C++ Headers */
#include<thread>
#include<chrono>

void callback(const mv_msgs::VehiclePoses& msg) { ROS_INFO_STREAM(msg); };

TEST(Constructor, stack)
{
  AgentPool a_pool(100, 5, 100);

  PositionPublisher("TopicOut", "robot1/odom", a_pool, false, std::string(), 5, 100);
}

TEST(Constructor, heap)
{
  AgentPool a_pool(100, 5, 100);

  PositionPublisher* test = new PositionPublisher("TopicOut", "robot1/odom", a_pool, false, std::string(), 5, 100);

  delete(test);
}

TEST(Get, full)
{
  AgentPool a_pool(100, 5, 100);

  PositionPublisher test("TopicOut", "robot1/odom", a_pool, false, std::string(), 5, 100);

  EXPECT_EQ("robot1/odom", test.getFrameId());
}

TEST(PublishPosition, full)
{
  ros::NodeHandle nh;
  AgentPool a_pool(1, 5, 100);
  PositionPublisher test("TopicOut", "robot1/odom", a_pool, false, std::string(), 5, 30);

  ros::Subscriber sub = nh.subscribe("TopicOut", 1, callback);

  ros::Rate print_rate(0.1);

  while(ros::ok())
  {
    ros::spinOnce();
    print_rate.sleep();
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_position_publisher_node");
  ros::NodeHandle m_nh;

  return RUN_ALL_TESTS();
}

/* test_position_publisher.cpp */

