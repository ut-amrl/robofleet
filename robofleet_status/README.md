# robofleet_status

Robofleet_status is an example ROS package that can be built in a Catkin workspace.  It can be installed on a machine running [robofleet_client](https://github.com/ut-amrl/robofleet_client/tree/master), which is part of the [Robofleet communication architecture](https://github.com/ut-amrl/robofleet).

Using the Robofleet communication architecture, robot clients communicate with a central [robofleet_server](https://github.com/ut-amrl/robofleet_server).  In addition, a web-based application, Webviz, visualizes robot client data.  A special message type, `amrl_msgs/RobofleetStatus`, is used by the Webviz to visualize this basic robot data.  This package can be added to a ROS catkin workspace and creates a rosnode called `robofleet_status` that publishes a `amrl_msgs/RobofleetStatus` message to a topic called `status`.  This package is designed to be simple enough that its contents could be integrated into a main robot ROS package.

## Dependencies

* [robofleet_client](https://github.com/ut-amrl/robofleet_client/tree/master)
* ROS Melodic
* A ROS catkin workspace

## Installation and Configuration

Instructions are for Ubuntu users:
1. `git clone` this repository into a catkin workspace.
2. Edit the example `src/robofleet_status_node.cpp` to subscribe to messages specific to your robot, such as power data, and use the contents of those messages to complete the `status_msg`.
3.  Edit the example `CMakeLists.txt` with the package dependencies for your robot's custom messages.
4.  Edit the example `package.xml` with your build and exec dependencies.

## Build
From your main catkin workspace directory, run 
`catkin build`

If the output of the build command indicates that you should source your workspace, do so:
`source ~/.bashrc`

## Usage

For your robot to appear in the Webviz, you should have an active websocket connection to the robofleet_server
and ROS should be running.  Ultimately it is ideal for these to be setup as services that run on robot startup.

* If ROS is not already started, open a terminal window and run the command `roscore`.

* Start your robofleet_client connection from the `robofleet_client` directory: `ROS_NAMESPACE="robot_name" make run`

* Your robot should be publishing messages on the topics you have used to build the RobofleetStatus message.
If it is not, start the relevant nodes or spoof the messages using [rostopic pub](http://wiki.ros.org/rostopic).

* **Run with**: `rosrun robofleet_status robofleet_status_node`

* Verify that your robot appears on the Webviz app at https://robofleet.csres.utexas.edu/robofleet/
