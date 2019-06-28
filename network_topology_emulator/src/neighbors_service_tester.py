#!/usr/bin/env python
"""Module to test the service to get neighbors from a NetworkEmulator class"""
from network_topology_emulator.srv import GetNeighbors
import rospy

def run():
    """Tests the service to get neighbors from a NetworkEmulator class"""
    get_neighbors = rospy.ServiceProxy('robot_neighbors', GetNeighbors)
    rospy.wait_for_service('robot_neighbors')
    try:
        while True:
            robot = raw_input("Enter the robot_id: ")
            print "Neighbors are {}".format(str(get_neighbors(robot)))

    except rospy.ServiceException as exc:
        print str(exc)

if __name__ == "__main__":
    run()
