# coding: utf-8
"""NatNet protocol parsing.

Copyright (c) 2017, Matthew Edwards.  This file is subject to the 3-clause BSD
license, as found in the LICENSE file in the top-level directory of this
distribution and at https://github.com/mje-nz/python_natnet/blob/master/LICENSE.
No part of python_natnet, including this file, may be copied, modified,
propagated, or distributed except according to the terms contained in the
LICENSE file.

Each message is implemented as a class with a serialize method and/or deserialize classmethod
(messages are not required to implement both).

To deserialize a packet, use :func:`~natnet.protocol.deserialize` and check the type of the return
value (against the message types you're interested in).  Alternatively, use
:func:`~natnet.protocol.deserialize_header` and check the message ID (against the message IDs you're
interested in), then use :func:`~natnet.protocol.deserialize_payload` to get a message instance."""

__all__ = [
    # Functions
    'set_version', 'serialize', 'deserialize', 'deserialize_header', 'deserialize_payload',
    # Misc
    'MessageId', 'Version',
    # Messages
    'ConnectMessage', 'DiscoveryMessage', 'EchoRequestMessage', 'EchoResponseMessage',
    'MocapFrameMessage', 'ModelDefinitionsMessage', 'RequestModelDefinitionsMessage', 'ServerInfoMessage']

from .common import (MessageId, Version, deserialize, deserialize_header, deserialize_payload,
                     serialize, set_version)
from .ConnectMessage import ConnectMessage
from .DiscoveryMessage import DiscoveryMessage
from .EchoRequestMessage import EchoRequestMessage
from .EchoResponseMessage import EchoResponseMessage
from .MocapFrameMessage import MocapFrameMessage
from .ModelDefinitionsMessage import ModelDefinitionsMessage
from .RequestModelDefinitionsMessage import RequestModelDefinitionsMessage
from .ServerInfoMessage import ServerInfoMessage
