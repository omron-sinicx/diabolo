# coding: utf-8
"""Crude NatNet server implementation for integration testing.

Copyright (c) 2017, Matthew Edwards.  This file is subject to the 3-clause BSD
license, as found in the LICENSE file in the top-level directory of this
distribution and at https://github.com/mje-nz/python_natnet/blob/master/LICENSE.
No part of python_natnet, including this file, may be copied, modified,
propagated, or distributed except according to the terms contained in the
LICENSE file.
"""
from __future__ import division, print_function

import select
import socket
import struct
import timeit

import attr

from . import Logger, protocol
from .__version__ import __version__
from .protocol import (ConnectMessage, DiscoveryMessage, EchoRequestMessage, EchoResponseMessage,
                       MocapFrameMessage, ModelDefinitionsMessage, RequestModelDefinitionsMessage,
                       ServerInfoMessage)
from .protocol.MocapFrameMessage import TimingInfo
from .protocol.ServerInfoMessage import ConnectionInfo


class ServerLogger(Logger):

    def _log_impl(self, msg, *args):
        print('Server:', msg % args)


@attr.s
class ServerConnection(object):

    _socket = attr.ib()  # type: socket.socket
    _multicast_address = ('239.255.42.100', 1511)  # Not the same as Motive default

    @classmethod
    def listen(cls, command_port=1510):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', command_port))

        # Set the time-to-live for messages to 1 so they do not go past the
        # local network segment.
        ttl = struct.pack('b', 1)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

        return cls(sock)

    def send_packet(self, packet, address=None):
        # Mostly same as Connection
        if address is None:
            address = self._multicast_address
        self._socket.sendto(packet, address)

    def send_message(self, message, address=None):
        # Same as Connection
        self.send_packet(protocol.serialize(message), address)

    def wait_for_packet_raw(self, timeout=None):
        # Mostly same as Connection
        sockets = [self._socket]
        readable, _, exceptional = select.select(sockets, [], sockets, timeout)
        if exceptional:
            raise IOError('Something went wrong with the socket')

        data = None
        client_address = None
        received_time = None
        if len(readable) > 0:
            # Just get the first message this time around
            data, client_address = readable[0].recvfrom(32768)  # type: bytes
            received_time = timeit.default_timer()  # type: float
        return data, client_address, received_time

    def wait_for_message(self, timeout=None):
        # Mostly same as Connection
        packet, client_address, received_time = self.wait_for_packet_raw(timeout)
        message = protocol.deserialize(packet) if packet is not None else None
        return message, client_address, received_time


class Server(object):

    """Fake server which implements just enough of the protocol for integration tests."""

    def __init__(self):
        self._conn = None
        self._last_frame_number = 0
        self._log = ServerLogger()
        self._last_frame_time = None
        self.should_exit = False

    def _send_server_info(self, client_address):
        connection_info = ConnectionInfo(
            data_port=self._conn._multicast_address[1],
            multicast=True,
            multicast_address=self._conn._multicast_address[0]
        )
        msg = ServerInfoMessage(
            app_name=u'python_natnet server',
            app_version=protocol.Version(*[int(i) for i in __version__.split('.')]),
            natnet_version=protocol.Version(3),
            high_resolution_clock_frequency=1000000000,
            connection_info=connection_info
        )
        self._conn.send_message(msg, client_address)

    def _send_echo_response(self, echo_request_message, client_address, received_time):
        msg = EchoResponseMessage(
            request_timestamp=echo_request_message.timestamp,
            received_timestamp=int(received_time*1e9)
        )
        self._conn.send_message(msg, client_address)

    def _send_model_definitions(self, client_address):
        msg = ModelDefinitionsMessage([])
        self._conn.send_message(msg, client_address)

    def _send_frame(self):
        now = timeit.default_timer()
        now_int = int(now*1e9)
        timing_info = TimingInfo(
            timecode=0,
            timecode_subframe=0,
            timestamp=now,
            camera_mid_exposure_timestamp=now_int,
            camera_data_received_timestamp=now_int,
            transmit_timestamp=now_int
        )
        self._last_frame_number += 1
        msg = MocapFrameMessage(
            frame_number=self._last_frame_number,
            markersets=[],
            rigid_bodies=[],
            skeletons=[],
            labelled_markers=[],
            force_plates=[],
            devices=[],
            timing_info=timing_info,
            params=0
        )
        self._conn.send_message(msg)
        self._last_frame_time = now

    def _run(self, rate):
        self._conn = ServerConnection.listen()
        self._log.info('Waiting for client to connect')
        while not self.should_exit:
            message, client_address, received_time = self._conn.wait_for_message(timeout=0.1)
            if message is None:
                continue
            if type(message) in (ConnectMessage, DiscoveryMessage):
                self._log.info('Sending server info to %s', client_address)
                self._send_server_info(client_address)
                break
            else:
                self._log.debug('Received message: %s', message)
        self._log.info('Streaming frames')
        while not self.should_exit:
            if self._last_frame_time:
                next_frame_due = self._last_frame_time + 1.0/rate
                sleep_time = max(0, next_frame_due - timeit.default_timer())
                message, client_address, received_time = self._conn.wait_for_message(timeout=sleep_time)
                if message is not None:
                    if type(message) is EchoRequestMessage:
                        self._send_echo_response(message, client_address, received_time)
                        continue
                    elif type(message) is RequestModelDefinitionsMessage:
                        self._send_model_definitions(client_address)
                        continue
                    else:
                        self._log.debug('Received message: %s', message)
            self._send_frame()
            self._log.debug('.')

    def run(self, rate=1):
        """Run the server.

        Args:
            rate (int): Rate at which to send mocap frames, in Hz
        """
        try:
            self._run(rate)
        except KeyboardInterrupt:
            pass
        finally:
            self._log.info('Exiting')


if __name__ == '__main__':
    s = Server()
    s.run()
