# Generated by the protocol buffer compiler.  DO NOT EDIT!

from google.protobuf import descriptor
from google.protobuf import message
from google.protobuf import reflection
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)



DESCRIPTOR = descriptor.FileDescriptor(
  name='time.proto',
  package='gazebo.msgs',
  serialized_pb='\n\ntime.proto\x12\x0bgazebo.msgs\"!\n\x04Time\x12\x0b\n\x03sec\x18\x01 \x02(\x05\x12\x0c\n\x04nsec\x18\x02 \x02(\x05')




_TIME = descriptor.Descriptor(
  name='Time',
  full_name='gazebo.msgs.Time',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    descriptor.FieldDescriptor(
      name='sec', full_name='gazebo.msgs.Time.sec', index=0,
      number=1, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='nsec', full_name='gazebo.msgs.Time.nsec', index=1,
      number=2, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  serialized_start=27,
  serialized_end=60,
)

DESCRIPTOR.message_types_by_name['Time'] = _TIME

class Time(message.Message):
  __metaclass__ = reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _TIME
  
  # @@protoc_insertion_point(class_scope:gazebo.msgs.Time)

# @@protoc_insertion_point(module_scope)