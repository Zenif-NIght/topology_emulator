# Topic Rebroadcaster

This package provides a ROS node that can be used to relay messages from one ROS topic
to anther at runtime.

## Usage

This package installs a launch file called
[topic_rebroadcaster.launch](./launch/topic_rebroadcaster.launch) located in
its launch directory. This launch file will startup the *topic_rebroadcaster_node*.

Once the *topic_rebroadcaster_node* is started it provides two service calls, one
to start relaying messages between two topics and another to stop relaying messages
between two topics. Both service definitions are generated in the
[rebroadcaster_msgs](./../rebroadcaster_msgs/) package and are called
[ConnectTopics.srv](./../rebroadcaster_msgs/srv/ConnectTopics.srv) and
[DisconnectRebroadcast.srv](./../rebroadcaster_msgs/srv/DisconnectRebroadcast.srv).


**Warning:** By nature nature ROS topics take a small amount of time to get set up
and pulled down. During this setup time some messages my be lost.

## Demo

For a demo of how to use this package you can look at the
[demo.cpp](./test/demo.cpp) node file and run the demo using the following command;

    roslaunch topic_rebroadcaster topic_rebroadcaster_demo.launch

