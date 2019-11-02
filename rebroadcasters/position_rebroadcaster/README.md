# Position Rebroadcaster

This package holds the position_rebroadcaster_node. This node can be used to
either package all the positions of ROS robots into one message or to publish only the
neighborhood set of each robot to that robot.

## Usage

To use the position_rebroadcaster_node first the node has to be running. The node can
be launched either using the provided launch file,
[position_rebroadcaster.launch](./launch/position_rebroadcaster.launch), or by adding that
launch file to a higher level launch file.

Once the node is launched it provides a service call to start publishing
[mv_msgs/VehiclePoses](./../../mv_msgs/msg/VehiclePoses.msg) messages which is described by
[rebroadcaster_msgs/ConnectPositionServer](./../rebroadcaster_msgs/srv/ConnectPositionServer.srv),
and a service call to end a pre-existing message topic described by
[rebroadcaster_msgs/DisconnectRebroadcast](./../rebroadcaster_msgs/srv/DisconnectRebroadcast.srv).

If the user wants to only publish the positions of the neighborhood set of the robot he or she
must make sure that the network_topology_emulator node is running and that the topic corresponding
to it is correct.

## Demos

To see a demo without the network_topology_emulator node run

    roslaunch position_rebroadcaster position_rebroadcaster_demo.launch

This will print a few messages published from the position_rebroadcaster_node.

If you want to see the published messages in Rvis go into the
[position_publisher.cpp](./src/position_publisher.cpp) file and set the variable VISUALIZE to
true. This will have each PositionPublisher publish nav_msgs/Odometry messages to the ROS
topic *visualize_transform_odom*. Once the position_rebroadcaster_node and a Rvis simulation
are running use the Rvis visualize functionality to print the *visualize_transform_odom*
topic in the simulation.

