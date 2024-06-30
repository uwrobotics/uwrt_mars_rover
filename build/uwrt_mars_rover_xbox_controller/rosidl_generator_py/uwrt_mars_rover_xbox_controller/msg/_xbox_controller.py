# generated from rosidl_generator_py/resource/_idl.py.em
# with input from uwrt_mars_rover_xbox_controller:msg/XboxController.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_XboxController(type):
    """Metaclass of message 'XboxController'."""

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
            module = import_type_support('uwrt_mars_rover_xbox_controller')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'uwrt_mars_rover_xbox_controller.msg.XboxController')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__xbox_controller
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__xbox_controller
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__xbox_controller
            cls._TYPE_SUPPORT = module.type_support_msg__msg__xbox_controller
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__xbox_controller

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class XboxController(metaclass=Metaclass_XboxController):
    """Message class 'XboxController'."""

    __slots__ = [
        '_drivetrain_joy_x',
        '_drivetrain_joy_y',
        '_gimble_joy_x',
        '_gimble_joy_y',
        '_lt',
        '_rt',
    ]

    _fields_and_field_types = {
        'drivetrain_joy_x': 'float',
        'drivetrain_joy_y': 'float',
        'gimble_joy_x': 'float',
        'gimble_joy_y': 'float',
        'lt': 'float',
        'rt': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.drivetrain_joy_x = kwargs.get('drivetrain_joy_x', float())
        self.drivetrain_joy_y = kwargs.get('drivetrain_joy_y', float())
        self.gimble_joy_x = kwargs.get('gimble_joy_x', float())
        self.gimble_joy_y = kwargs.get('gimble_joy_y', float())
        self.lt = kwargs.get('lt', float())
        self.rt = kwargs.get('rt', float())

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
        if self.drivetrain_joy_x != other.drivetrain_joy_x:
            return False
        if self.drivetrain_joy_y != other.drivetrain_joy_y:
            return False
        if self.gimble_joy_x != other.gimble_joy_x:
            return False
        if self.gimble_joy_y != other.gimble_joy_y:
            return False
        if self.lt != other.lt:
            return False
        if self.rt != other.rt:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def drivetrain_joy_x(self):
        """Message field 'drivetrain_joy_x'."""
        return self._drivetrain_joy_x

    @drivetrain_joy_x.setter
    def drivetrain_joy_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'drivetrain_joy_x' field must be of type 'float'"
        self._drivetrain_joy_x = value

    @property
    def drivetrain_joy_y(self):
        """Message field 'drivetrain_joy_y'."""
        return self._drivetrain_joy_y

    @drivetrain_joy_y.setter
    def drivetrain_joy_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'drivetrain_joy_y' field must be of type 'float'"
        self._drivetrain_joy_y = value

    @property
    def gimble_joy_x(self):
        """Message field 'gimble_joy_x'."""
        return self._gimble_joy_x

    @gimble_joy_x.setter
    def gimble_joy_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'gimble_joy_x' field must be of type 'float'"
        self._gimble_joy_x = value

    @property
    def gimble_joy_y(self):
        """Message field 'gimble_joy_y'."""
        return self._gimble_joy_y

    @gimble_joy_y.setter
    def gimble_joy_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'gimble_joy_y' field must be of type 'float'"
        self._gimble_joy_y = value

    @property
    def lt(self):
        """Message field 'lt'."""
        return self._lt

    @lt.setter
    def lt(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lt' field must be of type 'float'"
        self._lt = value

    @property
    def rt(self):
        """Message field 'rt'."""
        return self._rt

    @rt.setter
    def rt(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rt' field must be of type 'float'"
        self._rt = value
