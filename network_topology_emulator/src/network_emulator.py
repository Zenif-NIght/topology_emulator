#!/usr/bin/env python
"""Module containing the NetworkEmulator class definition."""
from __future__ import division

import sys
import argparse

from visualization_msgs.msg import MarkerArray, Marker
import rospy


from mv_msgs.msg import VehiclePoses, Neighbors
from network_topology_emulator.srv import GetNeighbors
from rebroadcaster_msgs.srv import ConnectPositionServer


class NetworkEmulator(object):
    """Base class for emulating a communication network in mulitvehicle robotics.

    This class is a ROS Node that receives the VehiclePoses message and builds
    a network that represents the communication between vehicles/robots.
    When inheriting from this class the most important function to override is
    the build_network funciton. The other functions should work without change
    as long as the network is defined correctly. If additional functionality
    is needed extending these functions is appropriate. For example, to store an
    additional member variable in __init__ by calling super(SomeInheritingClass,
    self).__init__(rate=some_rate_value) in the inheriting class's __init__. See
    DeltaDiskEmulator class for examples.
    """

    def __init__(self, rate=10):
        """Initialize the class with a rate (default of 10) in Hz."""
        # The network is stored as a mapping of robot_id:[robot_ids]
        self.network = {}

        # The mapping of robot:pose
        self.robots = {}

        # The publishers for each robot stored as a mapping of
        # robot_id:publisher
        self.robot_publishers = {}

        # Flag for if its the first time to indicate if publishers should be
        # initialized
        self.first_time = True

        # Save the rate in Hertz for use with vizualization
        self.rate_hz = rate

        # Set a frame id and namespace for use with vizualization
        self.frame_id = "map"
        self.viz_ns = "network_graph"

        ####################### ROS init stuff #################################
        rospy.init_node('network_emulator')
        rospy.Subscriber('robot_poses',
                         VehiclePoses,
                         self.robot_poses_received)
        self.rate = rospy.Rate(rate)
        self.viz_publisher = rospy.Publisher('network_graph',
                                             MarkerArray,
                                             queue_size=1)
        self.neighbors_service = rospy.Service("robot_neighbors",
                                               GetNeighbors,
                                               self.get_neighbors)
        ########################################################################

    def robot_poses_received(self, robot_poses):
        """Callback function for when VehiclePoses messages are received.

        Receive the VehiclePoses message and stores the data in the
        internal self.robots variable.
        """
        # intialize the robot dependant variables now that we have the robot_ids
        # only do this the first time
        if self.first_time:
            self.first_time = False
            for vehicle in robot_poses.vehicles:
                robot_id = vehicle.robot_id
                # create the publishers for each robot
                self.robot_publishers[robot_id] = \
                    rospy.Publisher('/{}/neighbors'.format(robot_id),
                                    Neighbors,
                                    queue_size=1)

        # update self.robots to reflect the current state of the network
        for vehicle in robot_poses.vehicles:
            robot_id = vehicle.robot_id
            pose = vehicle.pose
            self.robots[robot_id] = pose

    def build_network(self):
        """Update the emulation of the network topology.

        This is where the emulation of the different network types actually
        happens. Thus this method needs to be overridden by inheriting
        classes.
        """
        for robot in self.robots:
            self.network[robot] = self.robots.keys()

    def publish_network(self):
        """Publish the respective adjacency list to each robot's topic."""
        for robot in self.robot_publishers:
            pub = self.robot_publishers[robot]
            msg = Neighbors()
            msg.neighbors = self.network[robot]
            pub.publish(msg)

    def build_viz(self):
        """Build the network for vizualization.

        Build the network with arrow markers that point in the direction of
        information flow. In other words, if robot1 can "see" robot2 the arrow
        points from robot2 to robot1. Also adds small markers that indicate each
        robots position.
        """
        i = 1
        markers = MarkerArray()
        # create a marker and build a point array to mark the robot positions
        robot_marker = Marker()
        robot_marker.header.frame_id = self.frame_id
        robot_marker.header.stamp = rospy.Time.now()
        robot_marker.ns = self.viz_ns
        robot_marker.id = 0
        robot_marker.type = Marker.POINTS
        robot_marker.action = Marker.ADD
        robot_marker.scale.x = 0.15
        robot_marker.scale.y = 0.15
        robot_marker.color.a = 0.75
        robot_marker.color.r = 0.
        robot_marker.color.g = 1.
        robot_marker.color.b = 0.
        robot_marker.lifetime = rospy.Duration.from_sec(1/self.rate_hz)

        for robot in self.network:
            # Add each robot's position to the points list
            robot_marker.points.append(self.robots[robot].pose.position)

            for other_robot in self.network[robot]:
                # Create a marker and build all the aspects that are constant
                # for all the network arrows
                arrow_marker = Marker()
                arrow_marker.header.frame_id = self.frame_id
                arrow_marker.ns = self.viz_ns
                arrow_marker.type = Marker.ARROW
                arrow_marker.action = Marker.ADD
                arrow_marker.scale.x = 0.1
                arrow_marker.scale.y = 0.15
                arrow_marker.scale.z = 0.15
                arrow_marker.color.a = 0.5
                arrow_marker.color.r = 0.
                arrow_marker.color.g = 0.
                arrow_marker.color.b = 1.
                arrow_marker.lifetime = rospy.Duration.from_sec(1/self.rate_hz)

                # Build the marker details that are different for each arrow
                arrow_marker.header.stamp = rospy.Time.now()
                arrow_marker.id = i
                arrow_marker.points = [self.robots[other_robot].pose.position,
                                       self.robots[robot].pose.position]

                markers.markers.append(arrow_marker)
                i += 1

        markers.markers.append(robot_marker)
        return markers

    def get_neighbors(self, robot_request):
        """Service callback that returns a list of neighbors for a robot."""
        robot = robot_request.robot_id
        neighbors = self.network[robot]
        return Neighbors(neighbors)

    def run(self, vizualize=False):
        """Calculate the network topology and publishes at a fixed rate.

        Calculate the network topology based on the inheriting class's
        definition. Then publishes the topology to the robots. Uses the rate
        passed in at initialization to determine the rate at which to loop.
        """
        try:
            start_service = rospy.ServiceProxy("position_rebroadcaster/connect",
                                               ConnectPositionServer)
            start_service.call(topic="robot_poses",
                               frameId=self.frame_id,
                               filter=False)
        except rospy.ServiceException:
            rospy.loginfo("Service wasn't present. If another node is "
                          + "publishing that data will be used.")

        rospy.wait_for_message("robot_poses", VehiclePoses)
        while not rospy.is_shutdown():
            self.build_network()
            self.publish_network()
            if vizualize:
                markers = self.build_viz()
                self.viz_publisher.publish(markers)
            self.rate.sleep()


# pylint: disable=invalid-name
parser = argparse.ArgumentParser()
parser.add_argument("-r", "--rate",
                    default=10,
                    type=int,
                    help="The rate at which to publish the messages."
                    + " Default is 10Hz")
parser.add_argument("--viz",
                    action="store_true",
                    help="Enable vizualization")
if __name__ == '__main__':
    myargv = rospy.myargv(sys.argv)[1:]
    args = parser.parse_args(myargv)
    NetworkEmulator(rate=args.rate).run(vizualize=args.viz)
