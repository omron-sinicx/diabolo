# coding: utf-8
"""ModelDef message implementation.

Copyright (c) 2017, Matthew Edwards.  This file is subject to the 3-clause BSD
license, as found in the LICENSE file in the top-level directory of this
distribution and at https://github.com/mje-nz/python_natnet/blob/master/LICENSE.
No part of python_natnet, including this file, may be copied, modified,
propagated, or distributed except according to the terms contained in the
LICENSE file.

This message contains descriptions of all tracked models (rigid bodies, skeletons, markersets) and devices.

It is not sent automatically when tracked models changes. The next FrameOfData will have a flag set, then the client
sends a RequestModelDefinitions message to prompt the server to send this.
"""

__all__ = ['ModelDefinitionsMessage', 'MarkersetDescription', 'RigidBodyDescription', 'SkeletonDescription',
           'ForcePlateDescription', 'DeviceDescription']
try:
    # Only need this for type annotations
    from typing import Optional  # noqa: F401
except ImportError:
    pass

import enum
import warnings

import attr

from .common import (MessageId, SerDesRegistry, Version, int32_t, register_message, uint32_t,
                     vector3_t)


class ModelType(enum.IntEnum):
    MarkerSet = 0
    RigidBody = 1
    Skeleton = 2
    # I assume:
    ForcePlate = 3
    Device = 4
    # I wildly guess:
    Camera = 5


class ModelRegistry(SerDesRegistry):
    """Abuse SerDesRegistry a bit to use for model types."""

    @staticmethod
    def serialize(model):
        model_type = model.message_id
        payload = model.serialize()
        return uint32_t.pack(model_type) + payload

    def deserialize_header(*args, **kwargs):
        raise NotImplementedError()

    def deserialize_payload(*args, **kwargs):
        raise NotImplementedError()

    def deserialize(self, data, version=None, strict=None):
        """Deserialize a model description.

        Args:
            data (ParseBuffer): Pointer into a NatNet packet
            version (Version): Protocol version to use when deserializing
            strict (bool): ignored

        Returns:
            Model description instance
        """
        model_type = data.unpack(uint32_t)
        try:
            impl = self._implementation_types[model_type]
        except KeyError:
            warnings.warn('Unknown model description type {}'.format(model_type))
            # TODO: What's the right way to handle this?
            return None
        return impl.deserialize(data, version)


_registry = ModelRegistry()


@_registry.register_message(ModelType.MarkerSet)
@attr.s
class MarkersetDescription(object):

    """Description of a markerset.

    Attributes:
        name (str):
        marker_names: (list[str])
    """

    name = attr.ib()
    marker_names = attr.ib()

    @classmethod
    def deserialize(cls, data, version=None):
        name = data.unpack_cstr()
        marker_count = data.unpack(uint32_t)
        marker_names = [data.unpack_cstr() for i in range(marker_count)]
        return cls(name, marker_names)

    def serialize(self):
        return self.name.encode('utf-8') + b'\0' + uint32_t.pack(len(self.marker_names)) + \
               b''.join(m.encode('utf-8') + b'\0' for m in self.marker_names)


@_registry.register_message(ModelType.RigidBody)
@attr.s
class RigidBodyDescription(object):

    """Description of a rigid body.

    Attributes:
        name (str): Rigid body name if available
        id\_ (int): Streaming ID
        parent_id (int): For a rigid body which is part of a hierarchy (i.e., a skeleton), the ID of the parent rigid
            body
        offset_from_parent (tuple[float, float, float]): (x, y, z) offset relative to parent
        marker_positions (list[tuple[float, float, float]]): List of marker positions, if available
        required_active_labels (list[int]): List of expected active marker labels, if available
    """

    name = attr.ib()  # type: Optional[str]
    id_ = attr.ib()
    parent_id = attr.ib()
    offset_from_parent = attr.ib()
    marker_positions = attr.ib()  # type: Optional[list[float, float, float]]
    required_active_labels = attr.ib()

    @classmethod
    def deserialize(cls, data, version, skip_markers=None):
        name = None
        if version >= Version(2):
            name = data.unpack_cstr()

        id_ = data.unpack(int32_t)
        parent_id = data.unpack(int32_t)
        offset_from_parent = data.unpack(vector3_t)

        marker_positions = []
        required_active_labels = []

        # Marker positions are included from version 3, but if this is a skeleton bone description
        # then the count is zero.
        if version >= Version(3):
            marker_count = data.unpack(uint32_t)
            marker_positions = [data.unpack(vector3_t) for i in range(marker_count)]
            required_active_labels = [data.unpack(uint32_t) for i in range(marker_count)]

        return cls(name, id_, parent_id, offset_from_parent, marker_positions, required_active_labels)

    def serialize(self):
        return self.name.encode('utf-8') + b'\0' + int32_t.pack(self.id_) + \
               int32_t.pack(self.parent_id) + vector3_t.pack(*self.offset_from_parent) + \
               uint32_t.pack(len(self.marker_positions)) + \
               b''.join(vector3_t.pack(*m) for m in self.marker_positions) + \
               b''.join(uint32_t.pack(l) for l in self.required_active_labels)


@_registry.register_message(ModelType.Skeleton)
@attr.s
class SkeletonDescription(object):

    """Description of a skeleton.

    Attributes:
        name (str):
        id\_ (int): Streaming ID
        rigid_bodies (list[:class:`RigidBodyDescription`]):
    """

    name = attr.ib()
    id_ = attr.ib()
    rigid_bodies = attr.ib()

    @classmethod
    def deserialize(cls, data, version=None):
        name = data.unpack_cstr()
        id_ = data.unpack(int32_t)
        rigid_body_count = data.unpack(int32_t)
        rigid_bodies = [RigidBodyDescription.deserialize(data, version, skip_markers=True)
                        for i in range(rigid_body_count)]
        return cls(name, id_, rigid_bodies)


@_registry.register_message(ModelType.ForcePlate)
@attr.s
class ForcePlateDescription(object):

    id_ = attr.ib()
    serial_number = attr.ib()
    width = attr.ib()
    length = attr.ib()
    origin = attr.ib()
    calibration_matrix = attr.ib()
    corners = attr.ib()
    plate_type = attr.ib()
    channel_data_type = attr.ib()
    channels = attr.ib()
    channel_names = attr.ib()

    @classmethod
    def deserialize(cls, data, version=None):
        raise NotImplementedError


@_registry.register_message(ModelType.Device)
@attr.s
class DeviceDescription(object):

    id_ = attr.ib()  # type: int
    name = attr.ib()  # type: str
    serial_number = attr.ib()  # type: str
    device_type_ = attr.ib()  # type: int
    channel_data_type = attr.ib()  # type: int
    channel_names = attr.ib()  # type: list[str]

    @classmethod
    def deserialize(cls, data, version=None):
        raise NotImplementedError


@_registry.register_message(ModelType.Camera)
@attr.s
class CameraDescription(object):

    name = attr.ib()  # type: str

    @classmethod
    def deserialize(cls, data, version=None):
        # As far as I can tell the rest is garbage
        name = data.unpack_cstr(44)
        return cls(name)


@register_message(MessageId.ModelDef)
@attr.s
class ModelDefinitionsMessage(object):

    """Tracked model definitions.

    Attributes:
        models: Mixed list of :class:`MarkersetDescription`, :class:`RigidBodyDescription`,
            :class:`SkeletonDescription`, :class:`ForcePlateDescription`, and :class:`DeviceDescription`."""

    models = attr.ib()  # type: list

    @classmethod
    def deserialize(cls, data, version):
        models = []

        definition_count = data.unpack(uint32_t)
        for i in range(definition_count):
            models.append(_registry.deserialize(data, version))

        return cls(models)

    def serialize(self):
        return uint32_t.pack(len(self.models)) + b''.join(_registry.serialize(m) for m in self.models)
