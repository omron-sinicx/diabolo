# coding: utf-8
"""ServerInfo message implementation.

Copyright (c) 2017, Matthew Edwards.  This file is subject to the 3-clause BSD
license, as found in the LICENSE file in the top-level directory of this
distribution and at https://github.com/mje-nz/python_natnet/blob/master/LICENSE.
No part of python_natnet, including this file, may be copied, modified,
propagated, or distributed except according to the terms contained in the
LICENSE file.
"""
try:
    # Only need this for type annotations
    from typing import Optional  # noqa: F401
except ImportError:
    pass

import socket

import attr

from .common import MessageId, Version, bool_t, register_message, uint16_t, uint64_t


@attr.s
class ConnectionInfo(object):

    data_port = attr.ib()  # type: int
    multicast = attr.ib()  # type: bool
    multicast_address = attr.ib()  # type: bytes

    @classmethod
    def deserialize(cls, data, version):
        data_port = data.unpack(uint16_t)
        multicast = data.unpack(bool_t)
        multicast_address = socket.inet_ntoa(data.unpack_bytes(4))
        return cls(data_port, multicast, multicast_address)

    def serialize(self):
        multicast_address = socket.inet_aton(self.multicast_address)
        return uint16_t.pack(self.data_port) + bool_t.pack(self.multicast) + multicast_address


@register_message(MessageId.ServerInfo)
@attr.s
class ServerInfoMessage(object):

    app_name = attr.ib()  # type: str
    app_version = attr.ib()  # type: Version
    natnet_version = attr.ib()  # type: Version
    high_resolution_clock_frequency = attr.ib()  # type: Optional[int]
    connection_info = attr.ib()  # type: Optional[ConnectionInfo]

    @classmethod
    def deserialize(cls, data, version):
        """Deserialize a ServerInfo message.

        :type data: ParseBuffer
        :type version: Version"""

        app_name = data.unpack_cstr(256)
        app_version = Version.deserialize(data, version)
        natnet_version = Version.deserialize(data, version)

        high_resolution_clock_frequency = None
        connection_info = None
        if natnet_version >= Version(3):
            high_resolution_clock_frequency = data.unpack(uint64_t)
            connection_info = ConnectionInfo.deserialize(data, version)

        return cls(app_name, app_version, natnet_version, high_resolution_clock_frequency,
                   connection_info)

    def serialize(self):
        app_name = self.app_name.encode('utf-8')
        app_name += b'\0'*(256 - len(app_name))
        return app_name + self.app_version.serialize() + self.natnet_version.serialize() + \
            uint64_t.pack(self.high_resolution_clock_frequency) + self.connection_info.serialize()
