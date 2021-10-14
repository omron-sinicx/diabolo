# coding: utf-8
"""Utilities for protocol package.

Copyright (c) 2017, Matthew Edwards.  This file is subject to the 3-clause BSD
license, as found in the LICENSE file in the top-level directory of this
distribution and at https://github.com/mje-nz/python_natnet/blob/master/LICENSE.
No part of python_natnet, including this file, may be copied, modified,
propagated, or distributed except according to the terms contained in the
LICENSE file.
"""

import collections
import enum
import functools
import struct

import attr


class MessageId(enum.IntEnum):

    """Message IDs for each NatNet message (as in NatNetTypes.h).

    Attributes:
        Connect: Request for server info
        ServerInfo: Motive version, NatNet version, clock frequency, data port, and multicast
            address
        RequestModelDef: Request for model definitions
        ModelDef: List of definitions of rigid bodies, markersets, skeletons etc
        FrameOfData: Frame of motion capture data
        EchoRequest: Request server to immediately respond with its current time (used for clock
            sync)
        EchoResponse: Current server time (and time contained in EchoRequest message)
    """

    Connect = 0
    ServerInfo = 1
    Request = 2
    Response = 3
    RequestModelDef = 4
    ModelDef = 5
    RequestFrameOfData = 6
    FrameOfData = 7
    MessageString = 8
    Disconnect = 9
    KeepAlive = 10
    DisconnectByTimeout = 11
    EchoRequest = 12
    EchoResponse = 13
    Discovery = 14
    UnrecognizedRequest = 0x100  # NatNetTypes.h gives this as decimal 100, but that's incorrect


# Field types
bool_t = struct.Struct('?')
int16_t = struct.Struct('<h')
int32_t = struct.Struct('<i')
uint16_t = struct.Struct('<H')
uint32_t = struct.Struct('<I')
uint64_t = struct.Struct('<Q')
float_t = struct.Struct('<f')
double_t = struct.Struct('<d')
vector3_t = struct.Struct('<fff')
quaternion_t = struct.Struct('<ffff')


class ParseBuffer(object):

    """Buffer handling logic.

    Contains a buffer and an offset, and provides methods for unpacking data types (as struct.Struct
    instances) from the buffer."""

    def __init__(self, data):
        self.data = memoryview(data)
        self.offset = 0

    def __len__(self):
        """Length of remaining part of buffer."""
        return len(self.data) - self.offset

    # def __repr__(self):
    #     """Convenient string representation for debugging."""
    #     return ' '.join('{:02x}'.format(c) for c in self.data[self.offset:])

    def skip(self, struct_type, n=1):
        """Skip `n` fields of the given type."""
        self.offset += struct_type.size*n

    def unpack(self, struct_type):
        """Unpack a field.

        Args:
            struct_type (struct.Struct): Type of field to unpack
        """
        value = struct_type.unpack(self.data[self.offset:self.offset + struct_type.size])
        if len(value) == 1:
            value = value[0]
        self.offset += struct_type.size
        return value

    def unpack_cstr(self, size=None):
        """Unpack a null-terminated string field.

        If size is given then always unpack that many bytes, otherwise unpack up to the first null.
        """
        field = self.data[self.offset:]
        if size:
            field = self.data[self.offset:self.offset + size]

        # TODO: Would this be better?
        # value = data.split('\0', 1)[0]
        value, _, _ = field.tobytes().partition(b'\0')

        if size:
            self.offset += size
        else:
            self.offset += len(value) + 1
        return value.decode('utf-8')

    def unpack_bytes(self, size):
        """Unpack a fixed-length field of bytes."""
        value = self.data[self.offset:self.offset + size].tobytes()
        self.offset += size
        return value


class Version(collections.namedtuple('Version', ('major', 'minor', 'build', 'revision'))):

    """NatNet version, with correct comparison operator.

    Believe it or not, this is performance-critical.

    Attributes:
        major (int):
        minor (int):
        build (int):
        revision (int):"""

    _version_t = struct.Struct('BBBB')

    def __new__(cls, major, minor=0, build=0, revision=0):
        return super(Version, cls).__new__(cls, major, minor, build, revision)

    @classmethod
    def deserialize(cls, data, version=None):
        """Deserialize a Version from a ParseBuffer."""
        return cls(*data.unpack(cls._version_t))

    def serialize(self):
        """Serialize a Version to bytes."""
        return self._version_t.pack(*self)


@attr.s
class SerDesRegistry(object):

    """Registry of message implementations, which can serialize messages and deserialize packets.

    An instance of this is used to provide the module-level function."""

    _implementation_types = attr.ib(default=attr.Factory(dict))
    _version = attr.ib(default=Version(3))

    def register_message(self, id_):
        """Decorator to register the class which implements a given message.

        Args:
            id_ (:class:`MessageId`):
        """

        def register_message_impl(cls):
            cls.message_id = id_
            self._implementation_types[id_] = cls
            return cls

        return register_message_impl

    def set_version(self, version):
        """Set the NatNet protocol version used for deserialization.

        Args:
            version (Version): New version to use
        """
        self._version = version

    @staticmethod
    def serialize(message):
        """Serialize a message instance into a binary packet.

        Args:
            message: A message instance

        Returns:
            bytes: The message serialized as a packet, ready to be sent
        """
        message_id = message.message_id
        payload = message.serialize()
        return uint16_t.pack(message_id) + uint16_t.pack(len(payload)) + payload

    @staticmethod
    def deserialize_header(data):
        """Deserialize a packet into message ID and payload.

        Args:
            data (bytes): A NatNet packet

        Returns:
            tuple[MessageId, ParseBuffer]: Message ID and raw payload
        """
        data = ParseBuffer(data)
        message_id = MessageId(data.unpack(uint16_t))
        length = data.unpack(uint16_t)
        assert len(data) == length, 'Header says payload has length {}, but actual length is {}'\
            .format(len(data), length)
        return message_id, data

    def deserialize_payload(self, message_id, payload_data, version=None, strict=False):
        """Deserialize the payload of a packet into a message instance.

        Args:
            message_id (MessageId)
            payload_data (ParseBuffer): raw payload
            version (Version): Protocol version to use when deserializing
            strict (bool): Raise an exception if there is data left in the buffer after parsing

        Returns:
            Message instance
        """
        if version is None:
            version = self._version
        message_type = self._implementation_types[message_id]
        message = message_type.deserialize(payload_data, version)
        if strict:
            name = message_id.name
            assert len(payload_data) == 0, \
                "{} bytes remaining after parsing {} message".format(len(payload_data), name)
        return message

    def deserialize(self, data, version=None, strict=False):
        """Deserialize a packet into a message instance.

        Args:
            data (bytes): A NatNet packet
            version (Version): Protocol version to use when deserializing
            strict (bool): Raise an exception if there is data left in the buffer after parsing.

        Returns:
            Message instance
        """
        if version is None:
            version = self._version
        message_id, payload_data = self.deserialize_header(data)
        return self.deserialize_payload(message_id, payload_data, version, strict)


_registry = SerDesRegistry()


# Wrap these so sphinx documents them as proper functions
@functools.wraps(_registry.register_message)
def register_message(*args, **kwargs):
    return _registry.register_message(*args, **kwargs)


@functools.wraps(_registry.set_version)
def set_version(*args, **kwargs):
    return _registry.set_version(*args, **kwargs)


@functools.wraps(_registry.serialize)
def serialize(*args, **kwargs):
    return _registry.serialize(*args, **kwargs)


@functools.wraps(_registry.deserialize_header)
def deserialize_header(*args, **kwargs):
    return _registry.deserialize_header(*args, **kwargs)


@functools.wraps(_registry.deserialize)
def deserialize(*args, **kwargs):
    return _registry.deserialize(*args, **kwargs)


@functools.wraps(_registry.deserialize_payload)
def deserialize_payload(*args, **kwargs):
    return _registry.deserialize_payload(*args, **kwargs)
