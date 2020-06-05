# msg2fbs.py

This utility is designed to generate a [Flatbuffers][flatbuffers] [schema][fbs] (.fbs) from ROS [message types][ros msg]. The schema should be compiled with [`flatc`][flatc] to generate code for inclusion in other projects.

This enables the use of Flatbuffers to serialize and deserialize an arbitrary set of ROS messages in many languages.

## Example usage

1. `./msg2fbs.py --gen-constants -o schema.fbs std_msgs/String sensor_msgs/NavSatFix`
    * Generates definitions for the two given message types, as well as any dependencies
    * Generates tables with values for constants for each message type
    * **Note** that all desired message types should be generated into a single schema
2. `flatc --gen-object-api -c -T schema.fbs`
    * Generates Flatbuffers code for both C++ and TypeScript, with the Object API enabled
    * `schema_generated.h` and `schema_generated.ts` are then included and used in other projects

## Details

The basic idea of this project is to convert arbitrary ROS message definitions to Flatbuffers schemas, for wire encoding of ROS messages.

We also add metadata fields to each generated message type. Metadata is required to support various features of RoboFleet. Currently, we encode the message topic, which allows:
* Subscription to messages for a particular topic
* Identification of message type
* Identification of which client sent the message

* Namespaces message tables: `sensor_msgs/NavSatFix` -> `base_namespace.sensor_msgs.NavSatFix`
    * `base_namespace` avoids conflict with ROS types, and is configurable
* Generates one `table` for each message type
    * Contains all message fields
    * Automatically generates message types that are dependencies
    * Optionally generates an extra table `MsgTypeConstants` for each `MsgType`
        * Encodes any ROS message constants as Flatbuffers table fields with default values
        * Uses a potentially-unstable ROS API, isolated to `msg_util.py`
* Generates metadata definitions
    * `table MsgMetadata` - metadata table for all messages
        * Inserts a `__metadata: MsgMetadata` field at the start of each table, allowing metadata inspection for any message type
    * `table MsgWithMetadata` - a "base class" containing only the `__metadata` field, for accessing metadata of arbitrary message type
* Generates supporting definitions
    * `struct RosTime` - represents ROS `Time`s, with `secs` and `nsecs`
    * `struct RosDuration` - represents ROS `Duration`s, with `secs` and `nsecs`

## Backwards Compatibility

* Flatbuffers supports backwards compatibility (read new-version messages with old-version code) as long as new fields are only added to the end of a table definition. 
    * `msg2fbs` generates Flatbuffers fields in the same order as they originally appear in ROS message definitions.

[fbs]: https://google.github.io/flatbuffers/flatbuffers_guide_writing_schema.html
[flatbuffers]: https://google.github.io/flatbuffers/index.html
[flatc]: https://google.github.io/flatbuffers/flatbuffers_guide_using_schema_compiler.html
[ros msg]: http://wiki.ros.org/msg

