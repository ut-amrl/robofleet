# Robofleet

![Overview diagram](https://github.com/ut-amrl/robofleet/blob/1fa0a7acb2895eed4e452c627a4d8cc2d26e241f/overview_diagram.png)

This project aims to create a system that enables [ROS][ros]-based robot-to-robot communication, as well as visualization of robot data via a web-based frontend. Robots will run a client application to exchange data with a central server.

Each physical robot has existing control software, which publishes messages to the robot’s [ROS topics][ros topics]. The Robofleet client will exchange these existing messages with the Robofleet server. Using Robofleet, this software may also subscribe to ROS topics for information from other robots or control messages.

This project targets the ROS 1 framework.

## Usage

You will need to set up different components of Robofleet depending on your use case.

Make sure to clone submodules, using `git clone --recursive https://github.com/ut-amrl/robofleet`.

### Add a robot

`robofleet_client` enables a robot to exchange ROS messages with `robofleet_server`. To set up a new robot with Robofleet:
1. Clone recursively `robofleet_client`
2. Choose a [ROS Namespace][namespaces] for the robot (e.g. `/ut/testbot`)
3. Follow the configure and build instructions in the `robofleet_client` `README` file
   * Make sure to read the inline documentation about configuring message types and topics.
4. Extend an existing ROS node to publish `amrl_msgs/RobofleetStatus` messages to the `status` topic. This will list your robot in the web visualizer, though it is not strictly necessary.
   * Alternatively, use `robofleet_client/scripts/basic_status.py` to quickly list your robot.
5. Run the `robofleet_client` in your robot's namespace (e.g. `ROS_NAMESPACE="/ut/testbot" make run`)
6. Grant your robot permissions in the config file for `robofleet_server`
7. When trying to view your robot details in the web visualizer, remember to sign in if necessary!

### Run the server

`robofleet_server` enables broadcasting of ROS messages between many clients (robots and visualizers) over the network. To run the server:
1. Clone `robofleet_server` and follow the build instructions in its `README` file
2. Start the server with `yarn start`

### Use the web visualizer

`robofleet_webviz` is a browser-based Robofleet client that presents a GUI for many kinds of ROS messages received from `robofleet_server`. A special `RobofleetStatus` message enables a listing of all connected robots. To run the web visualizer:
1. Clone `robofleet_webviz` and follow the build instructions in its `README` file
2. Run the development server with `yarn start`

### Use the robot client and webviz without a central server

See information about "direct mode" in the `robofleet_client` `README`.

## Development

Most development tasks are isolated to each component of Robofleet. However, there are some tasks that require coordination across all components.

### Add a new message type (~30 mins)

To exchange ROS messages over the network, with support for message metadata and unusual kinds of clients (such as the browser), Robofleet needs a structured message encoding format. Robofleet currently encodes messages using [Flatbuffers][flatbuffers]. For details on how ROS messages are encoded in Flatbuffers, see the `README` for `msg2fbs`.

Imagine that you want to visualize some new message type, `my_viz/Marker`. First, build each of the Robofleet components as explained in each of their `README` files. Then, follow these steps:
1. `cd msg2fbs`
   1. Add `amrl_msgs` to `ROS_PACKAGE_PATH` and ensure that it has been built.
      `export ROS_PACKAGE_PATH=../robofleet_client/amrl_msgs:$ROS_PACKAGE_PATH`
      1. If your new message type is also in a local (not installed) package, make sure that it is also in your path.
      2. You can use another copy of `amrl_msgs`, but you must ensure that both copies stay in sync.
   2. Add the ROS name of the message type (e.g. `my_viz/Marker`) to the top of the `makefile`
   3. `make` to generate code and copy it into each Robofleet component.
2. `cd ../robofleet_client`
   1. Edit `src/config.hpp` to call `register_msg_type()` once per message type and topic, as shown in the example configuration. 
      For example: `register_msg_type<my_viz::Marker>("/my/topic", "my/topic")`
      Remember to include the ROS headers for your new message type.
      * Make sure to read the inline documentation.
   2. Edit `encode.hpp` to specialize `encode<>()` for your new message type. See existing specializations for examples. 
      * It helps to set up autocompletion in your editor.
      * Robofleet only uses the `metadata` attribute on the root message type. For example, if `my_viz/Marker` contains a `std_msgs/String`, you should set the `metadata` attribute of the `std_msgs/String` to `0` (null).
      * Remember that you can use `encode<>()` recursively using the provided helper functions.
      * Make sure to return the raw offset (the `.o` attribute) of your root message object.
   3. Edit `decode.hpp` to specialize `decode<>()` for your new message type.
      * Again, you can decode recursively using provided helper functions.
   4. `make` to test your changes
      * Linker errors generally indicate that you did not correctly specialize `encode<>()` or `decode<>()`.
   5. Add an example that works for normal usage to `src/config.example.hpp`
   6. `make format` to clean up your code
   7. Commit the changes.
3. `cd ../robofleet_server`
   1. `yarn build` to test the new schema code.
   2. Commit the changes.
4. `cd ../robofleet_webviz`
   1. `yarn build` to test the new schema code.
   2. Commit the changes.
5. `cd ..`
   1. Add the updated submodule commits and the changes to `msg2fbs`.
   2. Commit the changes.

All done! Now, you can consume the new message type in `robofleet_webviz`, as well as transmit and receive it with the `robofleet_client`.

## Future changes

When JavaScript support for [Flexbuffers][flexbuffers] is [added][flexbuffers js], it would be possible to switch from Flatbuffers to Flexbuffers encoding with a mostly-unchanged architecture. This would:
* Eliminate the need to maintain a schema and generated code
* Move the burden of schema conformance into application code (no schemas means implicit schemas, defined by how message fields are accessed in code.)
* Allow the encoding of all message types automatically by the robot client*
    * handling dynamically-typed ROS messages in C++ is probably possible [using ShapeShifter][generic subscriber] and [ros_msg_parser][ros_msg_parser] (formerly ros_type_introspection)
    * `msg2fbs` demonstrates how to encode arbitrary message types using `rospy`

[ros]: https://www.ros.org/
[ros topics]: http://wiki.ros.org/Topics
[namespaces]: http://wiki.ros.org/Names#Names-1
[remapping]: http://wiki.ros.org/Remapping%20Arguments
[relative names]: http://wiki.ros.org/Names#Resolving
[flatbuffers]: https://google.github.io/flatbuffers/
[flexbuffers]: https://google.github.io/flatbuffers/flexbuffers.html
[flexbuffers js]: https://github.com/google/flatbuffers/issues/5949
[generic subscriber]: http://wiki.ros.org/ros_type_introspection/Tutorials/GenericTopicSubscriber 
[ros_msg_parser]: https://github.com/facontidavide/ros_msg_parser
