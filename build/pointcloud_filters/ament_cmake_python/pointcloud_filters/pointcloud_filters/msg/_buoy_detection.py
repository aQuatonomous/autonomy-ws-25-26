# generated from rosidl_generator_py/resource/_idl.py.em
# with input from pointcloud_filters:msg/BuoyDetection.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BuoyDetection(type):
    """Metaclass of message 'BuoyDetection'."""

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
            module = import_type_support('pointcloud_filters')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'pointcloud_filters.msg.BuoyDetection')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__buoy_detection
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__buoy_detection
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__buoy_detection
            cls._TYPE_SUPPORT = module.type_support_msg__msg__buoy_detection
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__buoy_detection

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class BuoyDetection(metaclass=Metaclass_BuoyDetection):
    """Message class 'BuoyDetection'."""

    __slots__ = [
        '_range',
        '_bearing',
        '_z_mean',
        '_num_points',
        '_lateral_extent',
        '_radial_extent',
        '_confidence',
    ]

    _fields_and_field_types = {
        'range': 'float',
        'bearing': 'float',
        'z_mean': 'float',
        'num_points': 'uint32',
        'lateral_extent': 'float',
        'radial_extent': 'float',
        'confidence': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.range = kwargs.get('range', float())
        self.bearing = kwargs.get('bearing', float())
        self.z_mean = kwargs.get('z_mean', float())
        self.num_points = kwargs.get('num_points', int())
        self.lateral_extent = kwargs.get('lateral_extent', float())
        self.radial_extent = kwargs.get('radial_extent', float())
        self.confidence = kwargs.get('confidence', float())

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
        if self.range != other.range:
            return False
        if self.bearing != other.bearing:
            return False
        if self.z_mean != other.z_mean:
            return False
        if self.num_points != other.num_points:
            return False
        if self.lateral_extent != other.lateral_extent:
            return False
        if self.radial_extent != other.radial_extent:
            return False
        if self.confidence != other.confidence:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property  # noqa: A003
    def range(self):  # noqa: A003
        """Message field 'range'."""
        return self._range

    @range.setter  # noqa: A003
    def range(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'range' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'range' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._range = value

    @builtins.property
    def bearing(self):
        """Message field 'bearing'."""
        return self._bearing

    @bearing.setter
    def bearing(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'bearing' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'bearing' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._bearing = value

    @builtins.property
    def z_mean(self):
        """Message field 'z_mean'."""
        return self._z_mean

    @z_mean.setter
    def z_mean(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z_mean' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'z_mean' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._z_mean = value

    @builtins.property
    def num_points(self):
        """Message field 'num_points'."""
        return self._num_points

    @num_points.setter
    def num_points(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_points' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'num_points' field must be an unsigned integer in [0, 4294967295]"
        self._num_points = value

    @builtins.property
    def lateral_extent(self):
        """Message field 'lateral_extent'."""
        return self._lateral_extent

    @lateral_extent.setter
    def lateral_extent(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lateral_extent' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'lateral_extent' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._lateral_extent = value

    @builtins.property
    def radial_extent(self):
        """Message field 'radial_extent'."""
        return self._radial_extent

    @radial_extent.setter
    def radial_extent(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'radial_extent' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'radial_extent' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._radial_extent = value

    @builtins.property
    def confidence(self):
        """Message field 'confidence'."""
        return self._confidence

    @confidence.setter
    def confidence(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'confidence' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'confidence' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._confidence = value
