# generated from rosidl_generator_py/resource/_idl.py.em
# with input from pointcloud_filters:srv/GetCloudWindow.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetCloudWindow_Request(type):
    """Metaclass of message 'GetCloudWindow_Request'."""

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
                'pointcloud_filters.srv.GetCloudWindow_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_cloud_window__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_cloud_window__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_cloud_window__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_cloud_window__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_cloud_window__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetCloudWindow_Request(metaclass=Metaclass_GetCloudWindow_Request):
    """Message class 'GetCloudWindow_Request'."""

    __slots__ = [
        '_window_sec',
        '_merged',
    ]

    _fields_and_field_types = {
        'window_sec': 'float',
        'merged': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.window_sec = kwargs.get('window_sec', float())
        self.merged = kwargs.get('merged', bool())

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
        if self.window_sec != other.window_sec:
            return False
        if self.merged != other.merged:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def window_sec(self):
        """Message field 'window_sec'."""
        return self._window_sec

    @window_sec.setter
    def window_sec(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'window_sec' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'window_sec' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._window_sec = value

    @builtins.property
    def merged(self):
        """Message field 'merged'."""
        return self._merged

    @merged.setter
    def merged(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'merged' field must be of type 'bool'"
        self._merged = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_GetCloudWindow_Response(type):
    """Metaclass of message 'GetCloudWindow_Response'."""

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
                'pointcloud_filters.srv.GetCloudWindow_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_cloud_window__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_cloud_window__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_cloud_window__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_cloud_window__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_cloud_window__response

            from sensor_msgs.msg import PointCloud2
            if PointCloud2.__class__._TYPE_SUPPORT is None:
                PointCloud2.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetCloudWindow_Response(metaclass=Metaclass_GetCloudWindow_Response):
    """Message class 'GetCloudWindow_Response'."""

    __slots__ = [
        '_clouds',
        '_merged_cloud',
    ]

    _fields_and_field_types = {
        'clouds': 'sequence<sensor_msgs/PointCloud2>',
        'merged_cloud': 'sensor_msgs/PointCloud2',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'PointCloud2')),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'PointCloud2'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.clouds = kwargs.get('clouds', [])
        from sensor_msgs.msg import PointCloud2
        self.merged_cloud = kwargs.get('merged_cloud', PointCloud2())

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
        if self.clouds != other.clouds:
            return False
        if self.merged_cloud != other.merged_cloud:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def clouds(self):
        """Message field 'clouds'."""
        return self._clouds

    @clouds.setter
    def clouds(self, value):
        if __debug__:
            from sensor_msgs.msg import PointCloud2
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, PointCloud2) for v in value) and
                 True), \
                "The 'clouds' field must be a set or sequence and each value of type 'PointCloud2'"
        self._clouds = value

    @builtins.property
    def merged_cloud(self):
        """Message field 'merged_cloud'."""
        return self._merged_cloud

    @merged_cloud.setter
    def merged_cloud(self, value):
        if __debug__:
            from sensor_msgs.msg import PointCloud2
            assert \
                isinstance(value, PointCloud2), \
                "The 'merged_cloud' field must be a sub message of type 'PointCloud2'"
        self._merged_cloud = value


class Metaclass_GetCloudWindow(type):
    """Metaclass of service 'GetCloudWindow'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('pointcloud_filters')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'pointcloud_filters.srv.GetCloudWindow')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_cloud_window

            from pointcloud_filters.srv import _get_cloud_window
            if _get_cloud_window.Metaclass_GetCloudWindow_Request._TYPE_SUPPORT is None:
                _get_cloud_window.Metaclass_GetCloudWindow_Request.__import_type_support__()
            if _get_cloud_window.Metaclass_GetCloudWindow_Response._TYPE_SUPPORT is None:
                _get_cloud_window.Metaclass_GetCloudWindow_Response.__import_type_support__()


class GetCloudWindow(metaclass=Metaclass_GetCloudWindow):
    from pointcloud_filters.srv._get_cloud_window import GetCloudWindow_Request as Request
    from pointcloud_filters.srv._get_cloud_window import GetCloudWindow_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
