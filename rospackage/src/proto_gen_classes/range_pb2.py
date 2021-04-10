# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: range.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import proto_gen_classes.header_pb2 as header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='range.proto',
  package='',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x0brange.proto\x1a\x0cheader.proto\"l\n\x05Range\x12\x17\n\x06header\x18\x01 \x02(\x0b\x32\x07.Header\x12\x15\n\rfield_of_view\x18\x02 \x02(\x02\x12\x11\n\tmin_range\x18\x03 \x02(\x02\x12\x11\n\tmax_range\x18\x04 \x02(\x02\x12\r\n\x05range\x18\x05 \x02(\x02'
  ,
  dependencies=[header__pb2.DESCRIPTOR,])




_RANGE = _descriptor.Descriptor(
  name='Range',
  full_name='Range',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='Range.header', index=0,
      number=1, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='field_of_view', full_name='Range.field_of_view', index=1,
      number=2, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='min_range', full_name='Range.min_range', index=2,
      number=3, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='max_range', full_name='Range.max_range', index=3,
      number=4, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='range', full_name='Range.range', index=4,
      number=5, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
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
  serialized_start=29,
  serialized_end=137,
)

_RANGE.fields_by_name['header'].message_type = header__pb2._HEADER
DESCRIPTOR.message_types_by_name['Range'] = _RANGE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Range = _reflection.GeneratedProtocolMessageType('Range', (_message.Message,), {
  'DESCRIPTOR' : _RANGE,
  '__module__' : 'range_pb2'
  # @@protoc_insertion_point(class_scope:Range)
  })
_sym_db.RegisterMessage(Range)


# @@protoc_insertion_point(module_scope)
