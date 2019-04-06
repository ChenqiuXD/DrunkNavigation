# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: vision_detection.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='vision_detection.proto',
  package='',
  serialized_pb=_b('\n\x16vision_detection.proto\"\x9f\x01\n\x14Vision_DetectionBall\x12\r\n\x05vel_x\x18\x01 \x01(\x02\x12\r\n\x05vel_y\x18\x02 \x01(\x02\x12\x0c\n\x04\x61rea\x18\x03 \x01(\r\x12\t\n\x01x\x18\x04 \x02(\x02\x12\t\n\x01y\x18\x05 \x02(\x02\x12\x0e\n\x06height\x18\x06 \x01(\x02\x12\x12\n\nball_state\x18\x07 \x01(\r\x12\x12\n\nlast_touch\x18\x08 \x01(\r\x12\r\n\x05valid\x18\t \x02(\x08\"\x9a\x01\n\x15Vision_DetectionRobot\x12\x12\n\nconfidence\x18\x01 \x02(\x02\x12\x10\n\x08robot_id\x18\x02 \x01(\r\x12\t\n\x01x\x18\x03 \x02(\x02\x12\t\n\x01y\x18\x04 \x02(\x02\x12\x13\n\x0borientation\x18\x05 \x01(\x02\x12\r\n\x05vel_x\x18\x06 \x01(\x02\x12\r\n\x05vel_y\x18\x07 \x01(\x02\x12\x12\n\nrotate_vel\x18\x08 \x01(\x02\"\x99\x01\n\x15Vision_DetectionFrame\x12$\n\x05\x62\x61lls\x18\x01 \x02(\x0b\x32\x15.Vision_DetectionBall\x12-\n\rrobots_yellow\x18\x02 \x03(\x0b\x32\x16.Vision_DetectionRobot\x12+\n\x0brobots_blue\x18\x03 \x03(\x0b\x32\x16.Vision_DetectionRobot')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_VISION_DETECTIONBALL = _descriptor.Descriptor(
  name='Vision_DetectionBall',
  full_name='Vision_DetectionBall',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='vel_x', full_name='Vision_DetectionBall.vel_x', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='vel_y', full_name='Vision_DetectionBall.vel_y', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='area', full_name='Vision_DetectionBall.area', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='x', full_name='Vision_DetectionBall.x', index=3,
      number=4, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='y', full_name='Vision_DetectionBall.y', index=4,
      number=5, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='height', full_name='Vision_DetectionBall.height', index=5,
      number=6, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ball_state', full_name='Vision_DetectionBall.ball_state', index=6,
      number=7, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='last_touch', full_name='Vision_DetectionBall.last_touch', index=7,
      number=8, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='valid', full_name='Vision_DetectionBall.valid', index=8,
      number=9, type=8, cpp_type=7, label=2,
      has_default_value=False, default_value=False,
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
  oneofs=[
  ],
  serialized_start=27,
  serialized_end=186,
)


_VISION_DETECTIONROBOT = _descriptor.Descriptor(
  name='Vision_DetectionRobot',
  full_name='Vision_DetectionRobot',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='confidence', full_name='Vision_DetectionRobot.confidence', index=0,
      number=1, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='robot_id', full_name='Vision_DetectionRobot.robot_id', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='x', full_name='Vision_DetectionRobot.x', index=2,
      number=3, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='y', full_name='Vision_DetectionRobot.y', index=3,
      number=4, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='orientation', full_name='Vision_DetectionRobot.orientation', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='vel_x', full_name='Vision_DetectionRobot.vel_x', index=5,
      number=6, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='vel_y', full_name='Vision_DetectionRobot.vel_y', index=6,
      number=7, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='rotate_vel', full_name='Vision_DetectionRobot.rotate_vel', index=7,
      number=8, type=2, cpp_type=6, label=1,
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
  oneofs=[
  ],
  serialized_start=189,
  serialized_end=343,
)


_VISION_DETECTIONFRAME = _descriptor.Descriptor(
  name='Vision_DetectionFrame',
  full_name='Vision_DetectionFrame',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='balls', full_name='Vision_DetectionFrame.balls', index=0,
      number=1, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='robots_yellow', full_name='Vision_DetectionFrame.robots_yellow', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='robots_blue', full_name='Vision_DetectionFrame.robots_blue', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
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
  oneofs=[
  ],
  serialized_start=346,
  serialized_end=499,
)

_VISION_DETECTIONFRAME.fields_by_name['balls'].message_type = _VISION_DETECTIONBALL
_VISION_DETECTIONFRAME.fields_by_name['robots_yellow'].message_type = _VISION_DETECTIONROBOT
_VISION_DETECTIONFRAME.fields_by_name['robots_blue'].message_type = _VISION_DETECTIONROBOT
DESCRIPTOR.message_types_by_name['Vision_DetectionBall'] = _VISION_DETECTIONBALL
DESCRIPTOR.message_types_by_name['Vision_DetectionRobot'] = _VISION_DETECTIONROBOT
DESCRIPTOR.message_types_by_name['Vision_DetectionFrame'] = _VISION_DETECTIONFRAME

Vision_DetectionBall = _reflection.GeneratedProtocolMessageType('Vision_DetectionBall', (_message.Message,), dict(
  DESCRIPTOR = _VISION_DETECTIONBALL,
  __module__ = 'vision_detection_pb2'
  # @@protoc_insertion_point(class_scope:Vision_DetectionBall)
  ))
_sym_db.RegisterMessage(Vision_DetectionBall)

Vision_DetectionRobot = _reflection.GeneratedProtocolMessageType('Vision_DetectionRobot', (_message.Message,), dict(
  DESCRIPTOR = _VISION_DETECTIONROBOT,
  __module__ = 'vision_detection_pb2'
  # @@protoc_insertion_point(class_scope:Vision_DetectionRobot)
  ))
_sym_db.RegisterMessage(Vision_DetectionRobot)

Vision_DetectionFrame = _reflection.GeneratedProtocolMessageType('Vision_DetectionFrame', (_message.Message,), dict(
  DESCRIPTOR = _VISION_DETECTIONFRAME,
  __module__ = 'vision_detection_pb2'
  # @@protoc_insertion_point(class_scope:Vision_DetectionFrame)
  ))
_sym_db.RegisterMessage(Vision_DetectionFrame)


# @@protoc_insertion_point(module_scope)
