# coding: utf-8
"""Connect message implementation.

Copyright (c) 2017, Matthew Edwards.  This file is subject to the 3-clause BSD
license, as found in the LICENSE file in the top-level directory of this
distribution and at https://github.com/mje-nz/python_natnet/blob/master/LICENSE.
No part of python_natnet, including this file, may be copied, modified,
propagated, or distributed except according to the terms contained in the
LICENSE file.
"""

import attr

from .common import MessageId, Version, register_message


@register_message(MessageId.Connect)
@attr.s
class ConnectMessage(object):

    """Connect message (request ServerInfo from server)."""

    payload = attr.ib(default='')  # type: str

    # Not sure why there are two of these, but the SDK sets them both to (3, 0, 0, 0).  Could be min
    # and max protocol version supported?
    version1 = attr.ib(default=Version(3))  # type: Version
    version2 = attr.ib(default=Version(3))  # type: Version

    @classmethod
    def deserialize(cls, data, version):
        """Deserialize a Connect message.

        Args:
            data (ParseBuffer):
            version (Version)
        """

        # TODO: Check SDK implementation

        payload = data.unpack_cstr(256)
        version1 = Version.deserialize(data, version)
        version2 = Version.deserialize(data, version)
        return cls(payload, version1, version2)

    def serialize(self):
        return self.payload.encode('utf-8') + b'\0'*(256 - len(self.payload)) \
               + self.version1.serialize() + self.version2.serialize()
