# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: time.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='time.proto',
  package='',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\ntime.proto\"!\n\x04Time\x12\x0b\n\x03sec\x18\x01 \x02(\r\x12\x0c\n\x04nsec\x18\x02 \x02(\r'
)




_TIME = _descriptor.Descriptor(
  name='Time',
  full_name='Time',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='sec', full_name='Time.sec', index=0,
      number=1, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='nsec', full_name='Time.nsec', index=1,
      number=2, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=14,
  serialized_end=47,
)

DESCRIPTOR.message_types_by_name['Time'] = _TIME
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Time = _reflection.GeneratedProtocolMessageType('Time', (_message.Message,), {
  'DESCRIPTOR' : _TIME,
  '__module__' : 'time_pb2'
  # @@protoc_insertion_point(class_scope:Time)
  })
_sym_db.RegisterMessage(Time)


# @@protoc_insertion_point(module_scope)
