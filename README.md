# Robofleet

## Usage

You will need to set up different components of Robofleet depending on your use case.

### Add a robot

`robofleet_client` enables a robot to exchange ROS messages with `robofleet_server`. To set up a new robot with Robofleet:
1. Clone `robofleet_client` and follow the build instructions in its `README` file
2. **Unstable:** change the hard-coded server address at the top of `src/main.cpp`. In the future, configuration will be possible.
3. Choose a [ROS Namespace][namespaces] for the robot.
4. Extend an existing ROS node to publish `amrl_msgs/RobofleetStatus` messages to the `status` topic. This will list your robot in the web visualizer, though it is not strictly necessary.
    * Alternatively, use `robofleet_client/scripts/basic_status.py` to quickly list your robot.
5. Run your existing ROS nodes within the namespace--see [ROS Name Remapping][remapping]. The simplest method is to set the `ROS_NAMESPACE` environment variable.
    * Ensure that your ROS nodes are using [relative names][relative names] and not absolute names internally.
6. Run the `robofleet_client` in the same namespace (e.g. `ROS_NAMESPACE="/x/y/z" make run`)
* **Unstable:** currently, `robofleet_client` sends and receives all supported ROS message types. In the future, configuration will be possible.
* **Unstable:** currently, `robofleet_server` does not perform authentication. In the future, it will be necessary to set permissions for the robot.
* **Unstable:** currently, `robofleet_server` simply broadcasts each message to all clients. In the future, it will be possible to subscribe to messages on particular topics from other clients.

### Run the server

`robofleet_server` enables broadcasting of ROS messages between many clients (robots and visualizers) over the network. To run the server:
1. Clone `robofleet_server` and follow the build instructions in its `README` file
2. Start the server with `yarn start`
3. **Unstable:** currently, `robofleet_server` simply broadcasts messages to all clients. In the future, permission configuration will be necessary.

### Use the web visualizer

`robofleet_webviz` is a browser-based Robofleet client that presents a GUI for many kinds of ROS messages received from `robofleet_server`. A special `RobofleetStatus` message enables a listing of all connected robots. To run the web visualizer:
1. Clone `robofleet_webviz` and follow the build instructions in its `README` file
2. Run the development server with `yarn start`
* **Unstable:** in the future, a public build of the web visualizer will be accessible online.

## Development

Most development tasks are isolated to each component of Robofleet. However, there are some tasks that require coordination across all components.

### Add a message type

**Unstable** This section is subject to change. See _Future changes_. 

To exchange ROS messages over the network, with support for message metadata and unusual kinds of clients (such as the browser), Robofleet needs a structured message encoding format. Robofleet currently encodes messages using [Flatbuffers][flatbuffers]. For details on how ROS messages are encoded in Flatbuffers, see the `README` for `msg2fbs`.

Imagine that you want to visualize some new message type, `my_viz/Marker`. Follow these steps:
1. `cd msg2fbs`
    1. Add the ROS name of the message type to the top of the `makefile`.
    2. If your message is in a local ROS package (e.g. `amrl_msgs`), ensure that the package is in `ROS_PACKAGE_PATH`.
        * For example, `export ROS_PACKAGE_PATH=~/robofleet/robofleet_client/amrl_msgs:$ROS_PACKAGE_PATH`
            * This might not work for catkin packages
    3. Run `make`. This generates code and copies it into each Robofleet component.
2. `cd ../robofleet_client`
    1. Modify `src/main.cpp` to call `register_msg_type()` for the new ROS message type
    2. Specialize `encode()` in `encode.hpp` and `decode()` in `decode.hpp`
3. `cd ../robofleet_webviz`
    1. See existing code for examples of accessing message data

#### Future changes

When JavaScript support for [Flexbuffers][flexbuffers] is [added][flexbuffers js], it would be possible to switch from Flatbuffers to Flexbuffers encoding fairly seamlessly. This would:
* Eliminate the need to maintain a schema and generated code
* Allow the encoding of all message types automatically by the robot client*
    * handling dynamically-typed ROS messages in C++ is probably possible [using ShapeShifter][generic subscriber] and [ros_msg_parser][ros_msg_parser] (formerly ros_type_introspection)
* Move the burden of schema conformance into application code (no schemas means implicit schemas, defined by how message fields are accessed in code.)

[namespaces]: http://wiki.ros.org/Names#Names-1
[remapping]: http://wiki.ros.org/Remapping%20Arguments
[relative names]: http://wiki.ros.org/Names#Resolving
[flatbuffers]: https://google.github.io/flatbuffers/
[flexbuffers]: https://google.github.io/flatbuffers/flexbuffers.html
[flexbuffers js]: https://github.com/google/flatbuffers/issues/5949
[generic subscriber]: http://wiki.ros.org/ros_type_introspection/Tutorials/GenericTopicSubscriber 
[ros_msg_parser]: https://github.com/facontidavide/ros_msg_parser
