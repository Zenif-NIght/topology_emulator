#!/usr/bin/env python
"""This module contains a dummy publisher for publishing random VehiclePoses"""
import math
import random
import rospy
from mv_msgs.msg import VehiclePose, VehiclePoses


def run():
    """Creates a dummy publisher, calcualtes random poses, and publishes them.

    The random poses are limited by the value of "position_range" in the
    positive and negative directions. They also are all restricted to the z=0
    plane.
    """
    rospy.init_node('dummy')
    pub = rospy.Publisher('robot_poses', VehiclePoses, queue_size=1)
    position_range = 4


    while not rospy.is_shutdown():
        robots = VehiclePoses()
        for i in range(4):
            robot = VehiclePose()
            robot.pose.header.stamp = rospy.get_rostime()
            robot.pose.pose.position.x = random.uniform(-position_range,
                                                        position_range)
            robot.pose.pose.position.y = random.uniform(-position_range,
                                                        position_range)
            robot.pose.pose.position.z = 0
            u_1 = random.random()
            u_2 = random.random()
            u_3 = random.random()
            robot.pose.pose.orientation.w = math.sqrt(1-u_1)\
                                            *math.sin(2*math.pi*u_2)
            robot.pose.pose.orientation.x = math.sqrt(1-u_1)\
                                            *math.cos(2*math.pi*u_2)
            robot.pose.pose.orientation.y = math.sqrt(u_1)\
                                            *math.sin(2*math.pi*u_3)
            robot.pose.pose.orientation.z = math.sqrt(u_1)\
                                            *math.cos(2*math.pi*u_3)
            robot.robot_id = str(i)

            robots.vehicles.append(robot)

        robots.header.stamp = rospy.get_rostime()
        pub.publish(robots)
        rospy.sleep(5.)


if __name__ == "__main__":
    run()
