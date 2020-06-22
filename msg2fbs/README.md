# msg2fbs.py

This utility is designed to generate a [Flatbuffers][flatbuffers] [schema][fbs] (.fbs) from ROS [message types][ros msg]. The schema should be compiled with [`flatc`][flatc] to generate code for inclusion in other projects.

This enables the use of Flatbuffers to serialize and deserialize an arbitrary set of ROS messages in many languages.

## Updating flatbuffers code

A `makefile` is provided which automatically generates Flatbuffers code and copies it into each Robofleet component. The message types it generates can be edited in the `makefile`. Just run `make` to update generated code.

The `makefile` also builds the required `flatc` binary from Flatbuffers source included in the `flatbuffers` submodule using CMake.

## Example usage

1. `./msg2fbs.py --gen-enums -o schema.fbs amrl_msgs/RobofleetStatus sensor_msgs/NavSatFix`
    * Generates definitions for the two given message types, as well as any dependencies
    * Generates enums with values for constants for each message type
    * **Note** that all desired message types should be generated into a single schema
    * **Note** that you must set `ROS_PACKAGE_PATH` as noted in the amrl_msgs README to use `amrl_msgs` messages.
2. `flatc --gen-object-api --no-fb-import -c -T schema.fbs`
    * Generates Flatbuffers code for both C++ and TypeScript, with the Object API enabled
    * `schema_generated.h` and `schema_generated.ts` are then included and used in other projects
    * **Note** that we use the TypeScript generator, which emits `namespace` definitions that are not supported by Babel, and therefore by `create-react-app` applications. These files can only be compiled by `tsc` (or `ts-loader`) directly.

## Details

The basic idea of this project is to convert arbitrary ROS message definitions to Flatbuffers schemas, for wire encoding of ROS messages.

We also add metadata fields to the start of each generated message type. Metadata is required to support various features of RoboFleet. Since Flatbuffers supports backward-compatibility of message schemas, we can read the metadata fields without knowing the specific type of the message. Currently, we encode two metadata fields:
* `type` - the ROS name of the message type, e.g. `sensor_msgs/NavSatFix`
* `topic` - the topic of the message, e.g. `/navsat/fix`
* Both fields are needed because we include in the message topic a namespace for each robot.

### Namespaces

Each definition is namespaced based on its ROS name, for example: `sensor_msgs/NavSatFix` ⇒ `base_namespace.sensor_msgs.NavSatFix`.

`base_namespace` avoids conflict with ROS types, and is configurable.

### Message types

`msg2fbs` generates one `table` for each ROS message type.
* Contains a `__metadata` field
* Contains all ROS message fields

If a ROS message includes other ROS messages in its fields, these dependencies are also generated.

### Metadata

`msg2fbs` generates a `table MsgMetadata` definition in the base namespace. A `__metadata: MsgMetadata` field is inserted at the start of each table.

`msg2fbs` also generates a "base class" table called `MsgWithMetadata`. It contains only the `__metadata` field. Any type of message can be interpreted as a `MsgWithMetadata`.

### ROS msg constants

`msg2fbs` can optionally encode ROS message constants using one of the following modes:
* Enums mode: generates an enum type for each constant for each `MsgType`, namespaced in `MsgTypeConstants`
    * Only supports integral constants
    * Access is easier: in JS: `fb.sensor_msgs.NavSatStatusConstants.status_no_fix.value`
* Constant table mode: generates a table `MsgTypeConstants` for each `MsgType`
    * Encodes any ROS message constants as Flatbuffers table fields with default values
    * Access is more annoying: in JS: `fb.sensor_msgs.NavSatStatusConstants.getRootAsNavSatStatusConstants(new flatbuffers.ByteBuffer(new Uint8Array())).statusNoFix()`
* Note that ROS default values are currently generated as constants. If it seems useful, it may be possible to differentiate them from constants and generate actual default values in the output schemas.

This generator uses a potentially-unstable ROS API, isolated to `msg_util.py`.

### Supporting definitions

`msg2fbs` generates the following definitions to support all message types:
* `struct RosTime` - represents ROS `Time`s, with `secs` and `nsecs`
* `struct RosDuration` - represents ROS `Duration`s, with `secs` and `nsecs`

## Backwards Compatibility

* Flatbuffers supports backwards compatibility (read new-version messages with old-version code) as long as new fields are only added to the end of a table definition. 
    * `msg2fbs` generates Flatbuffers fields in the same order as they originally appear in ROS message definitions.

[fbs]: https://google.github.io/flatbuffers/flatbuffers_guide_writing_schema.html
[flatbuffers]: https://google.github.io/flatbuffers/index.html
[flatc]: https://google.github.io/flatbuffers/flatbuffers_guide_using_schema_compiler.html
[ros msg]: http://wiki.ros.org/msg

