#!/usr/bin/env python
import rospy
from mv_msgs.msg import VehiclePose, VehiclePoses
import math
import random


class DummyVehiclePosePublisher:
    def run(self):
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
                u1 = random.random()
                u2 = random.random()
                u3 = random.random()
                robot.pose.pose.orientation.w = math.sqrt(1-u1)\
                                                *math.sin(2*math.pi*u2)
                robot.pose.pose.orientation.x = math.sqrt(1-u1)\
                                                *math.cos(2*math.pi*u2)
                robot.pose.pose.orientation.y = math.sqrt(u1)\
                                                *math.sin(2*math.pi*u3)
                robot.pose.pose.orientation.z = math.sqrt(u1)\
                                                *math.cos(2*math.pi*u3)
                robot.robot_id = str(i)
            
                robots.vehicles.append(robot)
            
            robots.header.stamp = rospy.get_rostime()
            pub.publish(robots)
            rospy.sleep(5.)


if __name__ == "__main__":
    dummy = DummyVehiclePosePublisher()
    dummy.run()