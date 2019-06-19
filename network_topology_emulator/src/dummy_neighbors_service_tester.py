#!/usr/bin/env python
from network_topology_emulator.srv import GetNeighbors
import rospy

get_neighbors = rospy.ServiceProxy('robot_neighbors', GetNeighbors)
rospy.wait_for_service('robot_neighbors')
try:
    while(True):
        robot = raw_input("Enter the robot_id: ")
        print "Neighbors are {}".format(str(get_neighbors(robot)))

except rospy.ServiceException as exc:
    print str(exc)