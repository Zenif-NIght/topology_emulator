/**
 * @File: test_agent.cpp
 * @Date: 14 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the Agent object.
 **/

/* Google Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include"position_rebroadcasters/agent.hpp"

/* C++ Headers */
#include<thread>
#include<chrono>

nav_msgs::Odometry makeOdom()
{
  nav_msgs::Odometry obj;

  obj.header.seq              = 42;
  obj.header.stamp            = ros::Time::now();
  obj.header.frame_id         = "headerFrameId";
  obj.child_frame_id          = "child_frame_id";
  obj.pose.pose.position.x    = 1;
  obj.pose.pose.position.y    = 2;
  obj.pose.pose.position.z    = 3;
  obj.pose.pose.orientation.x = 4;
  obj.pose.pose.orientation.y = 5;
  obj.pose.pose.orientation.z = 6;
  obj.pose.pose.orientation.w = 7;

  return obj;
}

TEST(Constructor, stack)
{
  Agent("agentName", "odomTopic");
}

TEST(Constructor, heap)
{
  Agent* test = new Agent("AgentName", "odomTopic");

  delete(test);
}

TEST(GetPose, full)
{
  Agent test("agentName", "odomTopic");
  ros::NodeHandle m_nh;
  ros::Publisher pub = m_nh.advertise<nav_msgs::Odometry>("odomTopic", 5);

  EXPECT_NE(makeOdom().header.frame_id, test.getPose().pose.header.frame_id);
  
  pub.publish(makeOdom());

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  test.getLock().lock();
  EXPECT_EQ(makeOdom().header.frame_id, test.getPose().pose.header.frame_id);
  test.getLock().unlock();
}

TEST(Get, name)
{
  Agent test("agentName", "odomTopic");
  
  EXPECT_EQ(std::string("agentName"), test.getName());
}

TEST(Get, frameId)
{
  Agent test("agentName", "odomTopic");
  ros::NodeHandle m_nh;
  ros::Publisher pub = m_nh.advertise<nav_msgs::Odometry>("odomTopic", 5);

  EXPECT_EQ(std::string(), test.getFrameId());

  pub.publish(makeOdom());
  
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  EXPECT_EQ(std::string("headerFrameId"), test.getFrameId());
}

TEST(Get, Lock)
{
  Agent test("agentName", "odomTopic");
  
  test.getLock();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_agent_node");
  ros::NodeHandle m_nh;

  return RUN_ALL_TESTS();
}

/* test_agent.cpp */

