#! /usr/bin/env python3
"""
Utilities for generating Flatbuffers schemas for ROS message types. 
Requires rosbridge_library to be installed.

Schema formats documentation:
    http://wiki.ros.org/msg and 
    https://google.github.io/flatbuffers/flatbuffers_guide_writing_schema.html
"""

from argparse import ArgumentParser
import os
import re
import sys

import rospy
from rosbridge_library.internal.ros_loader import get_message_class

# types that are already defined in Flatbuffers
base_defined_types = {
    "bool",
    "int8",
    "int16",
    "int32",
    "int64",
    "uint8",
    "uint16",
    "uint32",
    "uint64",
    "float32",
    "float64",
    "string",
}

# direct ROS to Flatbuffers type remappings
type_mapping = {
    "time": "uint32",
    "duration": "int32"
}

def type_remap(ros_type_name):
    """ apply direct translations from ROS type names to flatbuffers type names """
    if ros_type_name in type_mapping:
        return type_mapping[ros_type_name]
    return ros_type_name

class Type:
    """
    Represents a data type. Constructed from a ROS type string.
    Produces Flatbuffers type strings.
    """
    def __init__(self, ros_type):
        match = re.match(r"^(?:([\w\/]+)\/)?(\w+)(\[\d*\])?$", ros_type)
        self.ros_type = ros_type
        if match.group(1) is not None:
            self.namespace = match.group(1).replace("/", ".")
        else:
            self.namespace = None
        self.name = match.group(2)
        self.is_array = match.group(3) is not None
    
    def fbs_type_name(self):
        """ Fully qualified Flatbuffers type name with remapping """
        if self.namespace is not None:
            n = "{}.{}".format(self.namespace, self.name)
        else: 
            n = self.name
        return type_remap(n)
    
    def fbs_type(self):
        """ Fully qualified Flatbuffers type with remapping """
        full_name = self.fbs_type_name()
        if self.is_array:
            return "[{}]".format(full_name)
        return full_name

    def is_defined(self, defined_types):
        """ Determine whether the given ROS type has been defined for Flatbuffers """
        return self.fbs_type_name() in defined_types

def gen_table(msg_type, items, defined_types):
    """ Generate namespaced .fbs table for given name, type pairs. """
    yield "namespace {};".format(msg_type.namespace)
    yield "table {} {{".format(msg_type.name)
    for k, v in items:
        yield "  {}:{};".format(k, v.fbs_type())
    yield "}"

def gen_msg(msg_type, defined_types):
    """ Generate .fbs definitions for a ROS message type, including dependencies. """
    # no need to generate already-defined type
    if msg_type.is_defined(defined_types):
        raise RuntimeError("Type already generated: {}".format(msg_type.ros_type))
    defined_types.add(msg_type.fbs_type_name())

    cls = get_message_class(msg_type.ros_type)

    name = cls._type
    keys = cls.__slots__
    types = [Type(name) for name in cls._slot_types]

    # generate dependency types
    for t in types:
        if not t.is_defined(defined_types):
            yield from gen_msg(t, defined_types)
    
    # generate table definition
    yield from gen_table(msg_type, zip(keys, types), defined_types)

def generate_schema(msg_type_names):
    """ Generate Flatbuffers .fbs schema for several ROS message types, including dependencies. """
    defined_types = set(base_defined_types)
    for msg_type_name in msg_type_names:
        yield from gen_msg(Type(msg_type_name), defined_types)

if __name__ == "__main__":
    ap = ArgumentParser("msg2fbs.py", description=__doc__)
    ap.add_argument("message_type", nargs="+",
        help="ROS names specifying which messages to generate (e.g. std_msgs/String)")
    ap.add_argument("--output-file", "-o", nargs="?", default=None,
        help="Specify an output file. Otherwise, the schema is written to stdout.")
    ns = ap.parse_args()
    
    if ns.output_file is None:
        output_file = sys.stdout
    else:
        output_file = open(ns.output_file, "w+")
    
    lines = generate_schema(ns.message_type)
    output_file.writelines(line + os.linesep for line in lines)
    output_file.close()
