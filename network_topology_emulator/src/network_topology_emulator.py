#!/usr/bin/env python
from mv_msgs.msg import VehiclePoses, Neighbors
from pdb import set_trace as pause
import rospy


class NetworkTopologyEmulator:
    def __init__(self, rate=10):
        # The network is stored as a mapping of robot_id:[robot_ids]
        self.network = {}

        # The list of all the robots in the network
        self.robots = {}
        
        # The publishers for each robot stored as a mapping of
        # robot_id:publisher
        self.robot_publishers = {}
        
        # Flag for if its the first time to indicate if publishers should be 
        # initialized
        self.first_time = True
        
        ####################### ROS init stuff #################################
        rospy.init_node('network_emulator')
        self.subscriber = rospy.Subscriber('robot_poses', 
                                            VehiclePoses, 
                                            self.robot_poses_received)
        self.rate = rospy.Rate(rate)
        ########################################################################
        
    def robot_poses_received(self, robot_poses):
        # intialize the robot dependant variables now that we have the robot_ids
        if self.first_time:
            self.first_time = False
            for vehicle in robot_poses.vehicles:
                robot_id = vehicle.robot_id
                # create the publishers for each robot
                self.robot_publishers[robot_id] = \
                    rospy.Publisher('{}/neighbors'.format(robot_id),
                                    Neighbors)
        
        # update self.robots to reflect the current state of the network
        for vehicle in robot_poses.vehicles:
            robot_id = vehicle.robot_id
            pose = vehicle.pose
            self.robots[robot_id] = pose
        self.network = self.build_network()

    def build_network(self):
        pass


if __name__ == '__main__':
    emulator = NetworkTopologyEmulator(10)
    rospy.spin()
