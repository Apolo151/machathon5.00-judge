# generated from rosidl_generator_py/resource/_idl.py.em
# with input from prius_msgs:msg/Control.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Control(type):
    """Metaclass of message 'Control'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'NO_COMMAND': 0,
        'NEUTRAL': 1,
        'FORWARD': 2,
        'REVERSE': 3,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('prius_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'prius_msgs.msg.Control')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__control
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__control
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__control
            cls._TYPE_SUPPORT = module.type_support_msg__msg__control
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__control

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'NO_COMMAND': cls.__constants['NO_COMMAND'],
            'NEUTRAL': cls.__constants['NEUTRAL'],
            'FORWARD': cls.__constants['FORWARD'],
            'REVERSE': cls.__constants['REVERSE'],
        }

    @property
    def NO_COMMAND(self):
        """Message constant 'NO_COMMAND'."""
        return Metaclass_Control.__constants['NO_COMMAND']

    @property
    def NEUTRAL(self):
        """Message constant 'NEUTRAL'."""
        return Metaclass_Control.__constants['NEUTRAL']

    @property
    def FORWARD(self):
        """Message constant 'FORWARD'."""
        return Metaclass_Control.__constants['FORWARD']

    @property
    def REVERSE(self):
        """Message constant 'REVERSE'."""
        return Metaclass_Control.__constants['REVERSE']


class Control(metaclass=Metaclass_Control):
    """
    Message class 'Control'.

    Constants:
      NO_COMMAND
      NEUTRAL
      FORWARD
      REVERSE
    """

    __slots__ = [
        '_header',
        '_throttle',
        '_brake',
        '_steer',
        '_shift_gears',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'throttle': 'double',
        'brake': 'double',
        'steer': 'double',
        'shift_gears': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.throttle = kwargs.get('throttle', float())
        self.brake = kwargs.get('brake', float())
        self.steer = kwargs.get('steer', float())
        self.shift_gears = kwargs.get('shift_gears', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.throttle != other.throttle:
            return False
        if self.brake != other.brake:
            return False
        if self.steer != other.steer:
            return False
        if self.shift_gears != other.shift_gears:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def throttle(self):
        """Message field 'throttle'."""
        return self._throttle

    @throttle.setter
    def throttle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'throttle' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'throttle' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._throttle = value

    @builtins.property
    def brake(self):
        """Message field 'brake'."""
        return self._brake

    @brake.setter
    def brake(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'brake' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'brake' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._brake = value

    @builtins.property
    def steer(self):
        """Message field 'steer'."""
        return self._steer

    @steer.setter
    def steer(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'steer' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'steer' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._steer = value

    @builtins.property
    def shift_gears(self):
        """Message field 'shift_gears'."""
        return self._shift_gears

    @shift_gears.setter
    def shift_gears(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'shift_gears' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'shift_gears' field must be an unsigned integer in [0, 255]"
        self._shift_gears = value
