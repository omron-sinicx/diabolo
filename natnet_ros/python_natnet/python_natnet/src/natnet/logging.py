# coding: utf-8
"""Logging implementation.

Copyright (c) 2017, Matthew Edwards.  This file is subject to the 3-clause BSD
license, as found in the LICENSE file in the top-level directory of this
distribution and at https://github.com/mje-nz/python_natnet/blob/master/LICENSE.
No part of python_natnet, including this file, may be copied, modified,
propagated, or distributed except according to the terms contained in the
LICENSE file.

The purpose of this class is to allow console output to be replaced with ROS logger calls in
natnet_ros."""

from __future__ import print_function


class Logger(object):

    """Dummy logger implementation that just calls print."""

    def _log_impl(self, msg, *args):
        """Implementation function to make subclassing work."""
        print(msg % args)

    def _log(self, msg, *args):
        """Print msg % args."""
        self._log_impl(msg, *args)

    debug = _log
    info = _log
    warning = _log
    error = _log
    fatal = _log
