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

/* C++ Headers */
#include<thread>
#include<chrono>

TEST(Constructor, stack)
{
 
}

TEST(Constructor, heap)
{

  delete(test);
}



int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_position_publisher_node");
  ros::NodeHandle m_nh;

  return RUN_ALL_TESTS();
}

/* test_position_publisher.cpp */

