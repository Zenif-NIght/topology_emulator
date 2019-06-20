/**
 * @File: test_relay.cpp
 * @Date: 18 June 2019
 * @Author: James Swedeen
 *
 * @brief
 * Tests the Relay object.
 **/

/* Google Headers */
#include<gtest/gtest.h>

/* ROS Headers */
#include<ros/ros.h>
#include<std_msgs/String.h>

/* Local Headers */
#include"topic_rebroadcaster/relay.hpp"

/* C++ Headers */
#include<chrono>
#include<thread>

TEST(Constructor, stack)
{
  Relay("topic_in", "topic_out");
}

TEST(Constructor, heap)
{
  Relay* test = new Relay("topic_in", "topic_out");

  delete(test);
}

TEST(Get, full)
{
  Relay test("topic_in", "topic_out");

  EXPECT_EQ("topic_in",  test.getInputTopic());
  EXPECT_EQ("topic_out", test.getOutputTopic());
}

std::string from_relay = "";

void callback(const std_msgs::StringConstPtr& msg_in) { from_relay = msg_in->data; }

TEST(Relay, full)
{
  Relay test("topic_in", "topic_out");

  ros::NodeHandle nh;
  ros::Publisher  pub = nh.advertise<std_msgs::String>(test.getInputTopic(),  1);
  ros::Subscriber sub = nh.subscribe<std_msgs::String>(test.getOutputTopic(), 1, callback);

  std_msgs::String to_relay;
  to_relay.data = "testing_testing_one_two_three";
  pub.publish(to_relay);

  while(std::string() == from_relay)
  {
    ros::spinOnce();
  }

  EXPECT_EQ("testing_testing_one_two_three", from_relay);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_relay_node");
  ros::NodeHandle m_nh;

  return RUN_ALL_TESTS();
}

/* test_relay.cpp */

