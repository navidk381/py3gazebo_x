# Generated by the protocol buffer compiler.  DO NOT EDIT!

from google.protobuf import descriptor
from google.protobuf import message
from google.protobuf import reflection
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)


import time_pb2

DESCRIPTOR = descriptor.FileDescriptor(
  name='log_status.proto',
  package='gazebo.msgs',
  serialized_pb='\n\x10log_status.proto\x12\x0bgazebo.msgs\x1a\ntime.proto\"\xa4\x02\n\tLogStatus\x12#\n\x08sim_time\x18\x01 \x01(\x0b\x32\x11.gazebo.msgs.Time\x12\x30\n\x08log_file\x18\x02 \x01(\x0b\x32\x1e.gazebo.msgs.LogStatus.LogFile\x1a\xbf\x01\n\x07LogFile\x12\x0b\n\x03uri\x18\x01 \x01(\t\x12\x11\n\tbase_path\x18\x02 \x01(\t\x12\x11\n\tfull_path\x18\x03 \x01(\t\x12\x0c\n\x04size\x18\x04 \x01(\x02\x12\x38\n\nsize_units\x18\x05 \x01(\x0e\x32$.gazebo.msgs.LogStatus.LogFile.Units\"9\n\x05Units\x12\t\n\x05\x42YTES\x10\x01\x12\x0b\n\x07K_BYTES\x10\x02\x12\x0b\n\x07M_BYTES\x10\x03\x12\x0b\n\x07G_BYTES\x10\x04')



_LOGSTATUS_LOGFILE_UNITS = descriptor.EnumDescriptor(
  name='Units',
  full_name='gazebo.msgs.LogStatus.LogFile.Units',
  filename=None,
  file=DESCRIPTOR,
  values=[
    descriptor.EnumValueDescriptor(
      name='BYTES', index=0, number=1,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='K_BYTES', index=1, number=2,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='M_BYTES', index=2, number=3,
      options=None,
      type=None),
    descriptor.EnumValueDescriptor(
      name='G_BYTES', index=3, number=4,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=281,
  serialized_end=338,
)


_LOGSTATUS_LOGFILE = descriptor.Descriptor(
  name='LogFile',
  full_name='gazebo.msgs.LogStatus.LogFile',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    descriptor.FieldDescriptor(
      name='uri', full_name='gazebo.msgs.LogStatus.LogFile.uri', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='base_path', full_name='gazebo.msgs.LogStatus.LogFile.base_path', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='full_path', full_name='gazebo.msgs.LogStatus.LogFile.full_path', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='size', full_name='gazebo.msgs.LogStatus.LogFile.size', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='size_units', full_name='gazebo.msgs.LogStatus.LogFile.size_units', index=4,
      number=5, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _LOGSTATUS_LOGFILE_UNITS,
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  serialized_start=147,
  serialized_end=338,
)

_LOGSTATUS = descriptor.Descriptor(
  name='LogStatus',
  full_name='gazebo.msgs.LogStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    descriptor.FieldDescriptor(
      name='sim_time', full_name='gazebo.msgs.LogStatus.sim_time', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='log_file', full_name='gazebo.msgs.LogStatus.log_file', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[_LOGSTATUS_LOGFILE, ],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  serialized_start=46,
  serialized_end=338,
)

_LOGSTATUS_LOGFILE.fields_by_name['size_units'].enum_type = _LOGSTATUS_LOGFILE_UNITS
_LOGSTATUS_LOGFILE.containing_type = _LOGSTATUS;
_LOGSTATUS_LOGFILE_UNITS.containing_type = _LOGSTATUS_LOGFILE;
_LOGSTATUS.fields_by_name['sim_time'].message_type = time_pb2._TIME
_LOGSTATUS.fields_by_name['log_file'].message_type = _LOGSTATUS_LOGFILE
DESCRIPTOR.message_types_by_name['LogStatus'] = _LOGSTATUS

class LogStatus(message.Message):
  __metaclass__ = reflection.GeneratedProtocolMessageType
  
  class LogFile(message.Message):
    __metaclass__ = reflection.GeneratedProtocolMessageType
    DESCRIPTOR = _LOGSTATUS_LOGFILE
    
    # @@protoc_insertion_point(class_scope:gazebo.msgs.LogStatus.LogFile)
  DESCRIPTOR = _LOGSTATUS
  
  # @@protoc_insertion_point(class_scope:gazebo.msgs.LogStatus)

# @@protoc_insertion_point(module_scope)