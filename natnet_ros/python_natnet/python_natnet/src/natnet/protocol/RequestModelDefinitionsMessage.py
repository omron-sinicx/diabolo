# coding: utf-8
"""RequestModelDef message implementation.

Copyright (c) 2017, Matthew Edwards.  This file is subject to the 3-clause BSD
license, as found in the LICENSE file in the top-level directory of this
distribution and at https://github.com/mje-nz/python_natnet/blob/master/LICENSE.
No part of python_natnet, including this file, may be copied, modified,
propagated, or distributed except according to the terms contained in the
LICENSE file.
"""

import attr

from .common import MessageId, register_message


@register_message(MessageId.RequestModelDef)
@attr.s
class RequestModelDefinitionsMessage(object):

    @classmethod
    def deserialize(cls, data=None, version=None):
        return cls()

    def serialize(self):
        return b''
