# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rover_msgs:msg/ControllerMsg.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ControllerMsg(type):
    """Metaclass of message 'ControllerMsg'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rover_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rover_msgs.msg.ControllerMsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__controller_msg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__controller_msg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__controller_msg
            cls._TYPE_SUPPORT = module.type_support_msg__msg__controller_msg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__controller_msg

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ControllerMsg(metaclass=Metaclass_ControllerMsg):
    """Message class 'ControllerMsg'."""

    __slots__ = [
        '_x',
        '_y',
        '_throttle',
        '_camerax',
        '_cameray',
        '_light',
        '_gear',
    ]

    _fields_and_field_types = {
        'x': 'float',
        'y': 'float',
        'throttle': 'float',
        'camerax': 'int32',
        'cameray': 'int32',
        'light': 'int32',
        'gear': 'int8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.throttle = kwargs.get('throttle', float())
        self.camerax = kwargs.get('camerax', int())
        self.cameray = kwargs.get('cameray', int())
        self.light = kwargs.get('light', int())
        self.gear = kwargs.get('gear', int())

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
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.throttle != other.throttle:
            return False
        if self.camerax != other.camerax:
            return False
        if self.cameray != other.cameray:
            return False
        if self.light != other.light:
            return False
        if self.gear != other.gear:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._x = value

    @builtins.property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._y = value

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
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'throttle' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._throttle = value

    @builtins.property
    def camerax(self):
        """Message field 'camerax'."""
        return self._camerax

    @camerax.setter
    def camerax(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'camerax' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'camerax' field must be an integer in [-2147483648, 2147483647]"
        self._camerax = value

    @builtins.property
    def cameray(self):
        """Message field 'cameray'."""
        return self._cameray

    @cameray.setter
    def cameray(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cameray' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'cameray' field must be an integer in [-2147483648, 2147483647]"
        self._cameray = value

    @builtins.property
    def light(self):
        """Message field 'light'."""
        return self._light

    @light.setter
    def light(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'light' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'light' field must be an integer in [-2147483648, 2147483647]"
        self._light = value

    @builtins.property
    def gear(self):
        """Message field 'gear'."""
        return self._gear

    @gear.setter
    def gear(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'gear' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'gear' field must be an integer in [-128, 127]"
        self._gear = value
