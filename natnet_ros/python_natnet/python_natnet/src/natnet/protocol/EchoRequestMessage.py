# coding: utf-8
"""EchoRequest message implementation.

Copyright (c) 2017, Matthew Edwards.  This file is subject to the 3-clause BSD
license, as found in the LICENSE file in the top-level directory of this
distribution and at https://github.com/mje-nz/python_natnet/blob/master/LICENSE.
No part of python_natnet, including this file, may be copied, modified,
propagated, or distributed except according to the terms contained in the
LICENSE file.
"""

import attr

from .common import MessageId, ParseBuffer, Version, register_message, uint64_t  # noqa: F401


@register_message(MessageId.EchoRequest)
@attr.s
class EchoRequestMessage(object):

    timestamp = attr.ib()  # type: int

    @classmethod
    def deserialize(cls, data, version):
        """Deserialize an EchoRequest message.

        :type data: ParseBuffer
        :type version: Version"""

        send_timestamp = data.unpack(uint64_t)
        return cls(send_timestamp)

    def serialize(self, version=None):
        return uint64_t.pack(self.timestamp)
