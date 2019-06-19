#!/usr/bin/env python
from network_emulator import NetworkEmulator
from visualization_msgs.msg import Marker
import rospy
import math

class DeltaDiskEmulator(NetworkEmulator):
    def __init__(self, rate=10, disk_radius=5):
        """ Initialize with a disk radius input (default is 1)

        Disk radius units mirror whatever units are used for the robot poses
        """
        super(DeltaDiskEmulator, self).__init__(rate)
        self.disk_radius = disk_radius

    def distance(self, position1, position2):
        """Calculate the distance between the two positions"""
        diffx = position1.x - position2.x
        diffy = position1.y - position2.y
        diffz = position1.z - position2.z

        return math.sqrt(diffx**2 + diffy**2 + diffz**2)

    def build_network(self):
        """Build the network with a delta disk topology

        Calculates the distance between robots and updates the network such
        that each robot only sees the robots to which it is "close enough"
        where "close enough" is given by the self.delta_radius parameter.
        """
        for robot, robot_pose in self.robots.items():
            neighbors = []
            for other_robot, other_robot_pose in self.robots.items():
                # calculate distance between robot and other_robot
                distance = self.distance(robot_pose.pose.position, 
                                 other_robot_pose.pose.position)
                # if its not me and close enough I can see it
                if robot != other_robot and distance <= self.disk_radius:
                    neighbors.append(other_robot)
            self.network[robot] = neighbors

    def build_viz(self):
        markers = super(DeltaDiskEmulator, self).build_viz()

        # define a sphere_list marker to mark all the robot disks
        marker = Marker()
        marker.header.frame_id = "/my_frame"
        marker.header.stamp = rospy.Time.now()
        marker.id = 1000
        marker.ns = "network_graph"
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 2 * self.disk_radius
        marker.scale.y = 2 * self.disk_radius
        marker.scale.z = 2 * self.disk_radius
        marker.color.a = 0.3
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.lifetime = rospy.Duration.from_sec(1/self.rate_hz)

        for robot in self.robots.keys():
            marker.points.append(self.robots[robot].pose.position)

        markers.markers.append(marker)

        return markers

if __name__ == "__main__":
    emulator = DeltaDiskEmulator(10, 5)
    emulator.run()
