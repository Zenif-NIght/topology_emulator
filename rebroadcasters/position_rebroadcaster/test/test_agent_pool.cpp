/**
 * @File: test_agent_pool.cpp
 * @Date: 14 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the AgentPool object.
 **/

/* Google Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include"position_rebroadcasters/agent_pool.hpp"

/* C++ Headers */
#include<thread>
#include<chrono>

TEST(Constructor, stack)
{
  AgentPool();
}

TEST(Constructor, heap)
{
  AgentPool* test = new AgentPool();

  delete(test);
}

TEST(getAgent, shouldWork)
{
  AgentPool test(100, 5, 100);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  EXPECT_EQ("robot1", test.getAgent("robot1")->getName());
}

TEST(getAgent, shouldntWork)
{
  AgentPool test(100, 5, 100);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  EXPECT_ANY_THROW(test.getAgent("robot2"));
}

TEST(getPose, shouldWork)
{
  AgentPool test(100, 5, 100);

  ros::NodeHandle m_nh;
  ros::Publisher odom_pub = m_nh.advertise<nav_msgs::Odometry>("robot3/odom", 5);
  nav_msgs::Odometry obj;
  obj.child_frame_id = "testing";
  odom_pub.publish(obj);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  EXPECT_EQ("testing", test.getPose("robot3"));
}

TEST(getPose, shouldWork)
{
  AgentPool test(100, 5, 100);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  EXPECT_ANY_THROW(test.getPose("robot3"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_agent_pool_node");
  ros::NodeHandle m_nh;

  ros::Publisher odom_pub = m_nh.advertise<nav_msgs::Odometry>("robot1/odom", 5);
  ros::Publisher not_odom_pub = m_nh.advertise<nav_msgs::Odometry>("robot1/notOdom", 5);

  return RUN_ALL_TESTS();
}

/* test_agent_pool.cpp */

