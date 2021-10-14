# coding: utf-8
"""Communications.

Copyright (c) 2017, Matthew Edwards.  This file is subject to the 3-clause BSD
license, as found in the LICENSE file in the top-level directory of this
distribution and at https://github.com/mje-nz/python_natnet/blob/master/LICENSE.
No part of python_natnet, including this file, may be copied, modified,
propagated, or distributed except according to the terms contained in the
LICENSE file.

Note that all local timestamps (e.g. packet reception time) are from :func:`timeit.default_timer`.
"""

import collections
import select
import socket
import struct
import timeit

import attr

from . import protocol
from .logging import Logger
from .protocol.MocapFrameMessage import LabelledMarker
from .protocol.ModelDefinitionsMessage import (MarkersetDescription, RigidBodyDescription,
                                               SkeletonDescription)
from .protocol.ServerInfoMessage import ConnectionInfo

__all__ = ['Client', 'Connection', 'TimestampAndLatency']


@attr.s
class Connection(object):

    """Connection to NatNet server.

    Attributes:
        last_sender_address (tuple[str, int]): Sending IP and port of last packet received.
    """

    _command_socket = attr.ib()  # type: socket.socket
    _data_socket = attr.ib()  # type: socket.socket
    _command_address = attr.ib()  # type: tuple[str, int]
    last_sender_address = attr.ib(None)

    def set_server_address(self, server=None, command_port=None):
        current_server, current_command_port = self._command_address
        command_address = (server or current_server, command_port or current_command_port)
        assert None not in command_address
        self._command_address = command_address

    def bind_data_socket(self, multicast_addr, data_port):
        """Bind data socket and begin receiving mocap frames.

        Args:
            multicast_addr (str): Server's IPv4 multicast address
            data_port (int): Server's data port
        """
        # Join multicast group
        mreq = struct.pack("4sl", socket.inet_aton(multicast_addr), socket.INADDR_ANY)
        self._data_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        # Bind to data port
        self._data_socket.bind(('', data_port))

    @classmethod
    def open(cls, server, command_port=1510, multicast_addr=None, data_port=None):
        """Open a connection to a NatNet server.

        If you don't know the multicast address and data port, you can get them from a ServerInfo
        message (by sending a Connect message and waiting) then open the data socket later with
        :func:`bind_data_socket`.

        Args:
            server (str): IPv4 address of server (hostname probably works too)
            command_port (int): Server's command port
            multicast_addr (str): Server's IPv4 multicast address
            data_port (int): Server's data port
        """
        # Create command socket and bind to any address
        command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        command_socket.bind(('', 0))
        command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # Create data socket
        data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        data_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        inst = cls(command_socket, data_socket, (server, command_port))

        if multicast_addr is not None and data_port is not None:
            inst.bind_data_socket(multicast_addr, data_port)
            # Otherwise, use a Connect message and get these from the ServerInfo response

        return inst

    def __del__(self):
        if self._command_socket:
            self._command_socket.close()
            self._command_socket = None
        if self._data_socket:
            self._data_socket.close()
            self._data_socket = None

    def wait_for_packet_raw(self, timeout=None):
        """Return the next packet to arrive on either socket as raw bytes.

        Args:
            timeout (float): Timeout in seconds

        Returns:
            tuple[bytes, float]: Raw packet and received timestamp, or (None, None) if a timeout
            occurred
        """
        sockets = [self._command_socket, self._data_socket]
        readable, _, exceptional = select.select(sockets, [], sockets, timeout)

        for s in exceptional:
            which = 'command' if s is self._command_socket else 'data'
            raise IOError('Something went wrong with the', which, 'socket')

        data = None
        received_time = None
        if len(readable) > 0:
            # Just get the first message this time around
            data, self.last_sender_address = readable[0].recvfrom(32768)  # type: bytes
            received_time = timeit.default_timer()  # type: float

        return data, received_time

    def wait_for_packet(self, timeout=None):
        """Return the next packet to arrive, deserializing the header but not the payload.

        If `timeout` is given and no packet is received within that time, return (None, None, None).

        Returns:
            tuple[MessageId, bytes, float]:
        """
        packet, received_time = self.wait_for_packet_raw(timeout)
        message_id, payload = protocol.deserialize_header(packet) if packet is not None else (None, None)
        return message_id, payload, received_time

    def wait_for_message(self, timeout=None):
        """Return the next message to arrive on either socket, or None if a timeout occurred."""
        packet, received_time = self.wait_for_packet_raw(timeout)
        message = protocol.deserialize(packet) if packet is not None else None
        return message, received_time

    def wait_for_message_with_id(self, id_, timeout=None):
        """Return the next message received of the given type, discarding any others."""
        # TODO: There's probably a better way of doing this (that doesn't throw away other packets)
        #       but it can wait until I see if that matters
        while True:
            message_id, payload, received_time = self.wait_for_packet(timeout)
            if message_id == id_:
                return protocol.deserialize_payload(message_id, payload), received_time

    def send_packet(self, packet):
        self._command_socket.sendto(packet, self._command_address)

    def send_message(self, message):
        self.send_packet(protocol.serialize(message))


@attr.s
class ClockSynchronizer(object):

    """Synchronize clocks with a NatNet server using Cristian's algorithm."""

    _server_info = attr.ib()
    _log = attr.ib()  # type: Logger
    _last_server_time = attr.ib(None)
    _last_synced_at = attr.ib(None)
    _min_rtt = attr.ib(1e-3)
    _echo_count = attr.ib(0)
    _last_sent_time = attr.ib(None)
    _skew = attr.ib(0)

    def initial_sync(self, conn):
        """Use a series of echoes to measure minimum round trip time.

        Args:
            conn (:class:`Connection`):
        """
        while self._echo_count < 100:
            self.send_echo_request(conn)
            response, received_time = conn.wait_for_message_with_id(protocol.MessageId.EchoResponse,
                                                                    timeout=0.1)
            if response is None:
                self._log.warning('Timeout out while waiting for echo response {}'
                                  .format(self._echo_count + 1))
            self.handle_echo_response(response, received_time)

    def server_ticks_to_seconds(self, server_ticks):
        return float(server_ticks)/self._server_info.high_resolution_clock_frequency

    def server_to_local_time(self, server_ticks):
        """Convert a NatNet HPC timestamp to local time (according to timeit.default_timer)."""
        server_time = self.server_ticks_to_seconds(server_ticks)
        server_time_since_last_sync = server_time - self._last_server_time
        local_time = self._last_synced_at + server_time_since_last_sync*(1 + self._skew)
        return local_time

    def local_to_server_time(self, local_time):
        local_time_since_last_sync = local_time - self._last_synced_at
        server_time = self._last_server_time + local_time_since_last_sync*(1 + self._skew)
        return server_time

    def server_time_now(self):
        """Get the current time on the server's HPC."""
        return self.local_to_server_time(timeit.default_timer())

    def send_echo_request(self, conn):
        self._last_sent_time = timeit.default_timer()
        sent_timestamp_int = int(self._last_sent_time*1e9)
        conn.send_message(protocol.EchoRequestMessage(sent_timestamp_int))

    def handle_echo_response(self, response, received_time):
        if response.request_timestamp != int(self._last_sent_time*1e9):
            self._log.warning('Warning: echo response does not match last echo request ' +
                              '(last sent at {}, received response for {})'
                              .format(self._last_sent_time*1e9, response.request_timestamp))
            return
        rtt = received_time - self._last_sent_time
        server_reception_time = self.server_ticks_to_seconds(response.received_timestamp)
        if self._last_server_time is None:
            # First echo, initialize
            self._last_server_time = server_reception_time + rtt/2
            self._last_synced_at = received_time
            self._log.debug('First echo: RTT {:.2f}ms, server time {:.1f}'
                            .format(1000*rtt, self._last_server_time))
        else:
            # The true server time falls within server_reception_time +- (rtt - true_min_rtt)/2.
            # We'd generally like to be within 0.1ms of the actual time, which would require the RTT
            # to be less than 0.1ms over the minimum RTT.  However, I've measured clock skew of
            # 0.03ms/s before, so if we start with a perfect estimate and don't sync for 5 seconds we
            # could already be out by 0.1ms.  Therefore our threshold should start at 0.05ms and
            # increase over time such that it's always a bit less than our potential accumulated
            # drift.
            # TODO: Kalman filter
            dt = received_time - self._last_synced_at
            accumulated_drift = dt*0.05e-3  # Assume skew is severe and we have a bad estimate of it
            rtt_threshold = self._min_rtt + max(0.05e-3, max(0.1e-3, accumulated_drift))
            if rtt < rtt_threshold:
                old_server_time_when_received = self.local_to_server_time(received_time)
                self._last_server_time = server_reception_time + rtt/2
                self._last_synced_at = received_time
                correction = self._last_server_time - old_server_time_when_received
                drift = correction/dt
                # This only works over a reasonably long time period
                if dt > 1:
                    if self._skew == 0:
                        # Initialize
                        self._skew = -drift
                    else:
                        # Slowly converge on the true skew
                        self._skew += drift/2
                self._log.debug(
                    ('Echo {: 5d}: RTT {:.2f}ms (min {:.2f}ms), server time {:.1f}s, ' +
                     'dt {: .3f}s, correction {: .3f}ms, drift {:7.3f}ms/s, new skew: {: .3f}ms/s')
                    .format(self._echo_count, 1000*rtt, 1000*self._min_rtt, self._last_server_time,
                            dt, 1000*correction, 1000*drift, 1000*self._skew))

        if rtt < self._min_rtt:
            self._min_rtt = rtt
        self._echo_count += 1

    def update(self, conn):
        now = timeit.default_timer()
        time_since_last_echo = now - self._last_sent_time
        time_since_last_sync = now - self._last_synced_at

        minimum_time_between_echo_requests = 0.5
        if time_since_last_sync > 5:
            minimum_time_between_echo_requests = 0.1
        if time_since_last_echo > minimum_time_between_echo_requests:
            self.send_echo_request(conn)


@attr.s
class TimestampAndLatency(object):

    """Timing information for a received mocap frame.

    Attributes:
        timestamp (float): Camera mid-exposure timestamp (according to local clock)
        system_latency (float): Time from camera mid-exposure to Motive transmitting frame
        transit_latency (float): Time from transmitting frame to receiving frame
        processing_latency (float): Time from receiving frame to calling callback
    """

    timestamp = attr.ib()  # type: float
    system_latency = attr.ib()  # type: float
    transit_latency = attr.ib()  # type: float
    processing_latency = attr.ib()  # type: float

    @classmethod
    def _calculate(cls, received_timestamp, timing_info, clock):
        """Calculate latencies and local timestamp.

        Args:
            received_timestamp (float):
            timing_info (:class:`~protocol.MocapFrameMessage.TimingInfo`):
            clock (:class:`ClockSynchronizer`):
        """
        if timing_info.camera_mid_exposure_timestamp is not None:
            timestamp = clock.server_to_local_time(timing_info.camera_mid_exposure_timestamp)
            system_latency_ticks = timing_info.transmit_timestamp - timing_info.camera_mid_exposure_timestamp
            system_latency = clock.server_ticks_to_seconds(system_latency_ticks)
            transit_latency = received_timestamp - clock.server_to_local_time(timing_info.transmit_timestamp)
            processing_latency = timeit.default_timer() - received_timestamp
            return cls(timestamp, system_latency, transit_latency, processing_latency)
        else:
            # TODO: Figure out what to do on v2
            timestamp = clock.server_to_local_time(timing_info.timestamp)
            return cls(timestamp, None, None, None)

    @property
    def latency(self):
        """Time from camera mid-exposure to calling callback."""
        if self.system_latency:
            return self.system_latency + self.transit_latency + self.processing_latency
        else:
            return None


class DiscoveryError(EnvironmentError):
    pass


@attr.s
class Client(object):

    """NatNet client.

    This class connects to a NatNet server and calls a callback whenever a frame of mocap data
    arrives.
    """

    _conn = attr.ib()  # type: Connection
    _clock_synchronizer = attr.ib()  # type: ClockSynchronizer
    _log = attr.ib()  # type: Logger
    _model_definitions = attr.ib(attr.Factory(list))  # type: list
    _expected_markers = attr.ib(attr.Factory(set))  # type: set
    _model_names = attr.ib(attr.Factory(dict))  # type: dict[int, str]
    _callback = attr.ib(None)
    _model_callback = attr.ib(None)

    @classmethod
    def _setup_client(cls, conn, server_info, logger):
        protocol.set_version(server_info.natnet_version)

        conn.bind_data_socket(server_info.connection_info.multicast_address,
                              server_info.connection_info.data_port)

        logger.debug('Synchronizing clocks')
        clock_synchronizer = ClockSynchronizer(server_info, logger)
        clock_synchronizer.initial_sync(conn)
        inst = cls(conn, clock_synchronizer, logger)

        logger.debug('Getting data descriptions')
        conn.send_message(protocol.RequestModelDefinitionsMessage())
        model_definitions_message, _ = conn.wait_for_message_with_id(protocol.MessageId.ModelDef)
        inst._handle_model_definitions(model_definitions_message)

        logger.info('Ready')
        return inst

    @classmethod
    def _discover_and_connect(cls, logger, timeout=None):
        logger.info('Discovering servers')
        conn = Connection.open('<broadcast>')
        conn.send_message(protocol.DiscoveryMessage())

        servers = []
        while True:
            info, _ = conn.wait_for_message(timeout=timeout)
            if info is None:
                # Timeout
                break
            address = conn.last_sender_address
            logger.info('Found server %s', address)
            logger.debug('Server application: %s', info.app_name)
            logger.debug('Server version: %s', info.app_version)
            assert info.connection_info.multicast
            servers.append((address, info))

        if not servers:
            raise DiscoveryError('No servers found')
        if len(servers) > 1:
            raise DiscoveryError('Multiple servers found, choose one manually')

        server_address, server_info = servers[0]
        conn.set_server_address(*server_address)
        return cls._setup_client(conn, server_info, logger)

    @classmethod
    def _simple_connect(cls, server, logger, timeout=None):
        logger.info('Connecting to %s', server)
        conn = Connection.open(server)

        logger.debug('Getting server info')
        conn.send_message(protocol.ConnectMessage())
        server_info, received_time = conn.wait_for_message_with_id(protocol.MessageId.ServerInfo,
                                                                   timeout=timeout)
        logger.debug('Server application: %s', server_info.app_name)
        logger.debug('Server version: %s', server_info.app_version)
        if server_info.natnet_version >= protocol.Version(3):
            # Unicast not supported
            assert server_info.connection_info.multicast
        else:
            # Before NatNet 3.0 the multicast address wasn't sent and you always had to specify it
            # manually.  For the sake of getting this to work, let's just assume connecting to the
            # default multicast address works.
            ci = ConnectionInfo(data_port=1511, multicast=True, multicast_address=b'239.255.42.99')
            logger.warning('Assuming server is in multicast mode on {}:{}'.format(
                           ci.data_port, ci.multicast_address))
            server_info.connection_info = ci

        return cls._setup_client(conn, server_info, logger)

    @classmethod
    def connect(cls, server=None, logger=Logger(), timeout=1):
        """Connect to a NatNet server.

        Raises :class:`DiscoveryError` if `server` is not provided and discovery fails.

        Args:
            server (str): IPv4 address of server (hostname probably works too), or None to
                autodiscover
            logger (:class:`~logging.Logger`):
            timeout (int): How long to wait for server(s) to respond
        """
        if server is None:
            return cls._discover_and_connect(logger, timeout)
        else:
            return cls._simple_connect(server, logger, timeout)

    def set_callback(self, callback):
        """Set the frame callback.

        It will be called with a list of :class:`~natnet.protocol.MocapFrameMessage.RigidBody`, a
        list of :class:`~natnet.protocol.MocapFrameMessage.Skeleton`, a list of
        :class:`~natnet.protocol.MocapFrameMessage.LabelledMarker`, and a :class:`~natnet.comms.TimestampAndLatency`.
        """
        self._callback = callback

    def _call_model_callback(self):
        if not self._model_callback:
            return
        rigid_body_descriptions = [m for m in self._model_definitions if type(m) is RigidBodyDescription]
        skeleton_descriptions = [m for m in self._model_definitions if type(m) is SkeletonDescription]
        markerset_descriptions = [m for m in self._model_definitions if type(m) is MarkersetDescription]
        self._model_callback(rigid_body_descriptions, skeleton_descriptions, markerset_descriptions)

    def set_model_callback(self, callback):
        """Set the model definition callback.

        It will be called with a list of :class:`~natnet.protocol.ModelDefinitionsMessage.RigidBodyDescription`, a list
        of :class:`~natnet.protocol.ModelDefinitionsMessage.SkeletonDescription`, and a list of
        :class:`~natnet.protocol.ModelDefinitionsMessage.MarkersetDescription`, immediately and whenever the tracked
        models change.
        """
        assert callback is not None
        self._model_callback = callback
        self._call_model_callback()

    def _do_occlusion_workaround(self, labelled_markers, markersets):
        """Work around strange "solver replaces occlusion" behaviour in Motive 2.0.

        "Solver replaces occlusion" does not do what you would expect.  When a marker is occluded,
        rather than streaming it with occluded=True and modelSolved=True, it is not streamed as a
        labelled marker at all.  Rather, "Solver replaces occlusion" enables streaming markersets,
        which are lists of all the markers in each rigid body as (effectively) unlabelled markers.
        To detect an occluded marker, we have to check if there are any markers missing in each
        rigid body and then find them in the markerset.  For sanity purposes, we hide this detail
        and make it look like they did the sensible thing.
        """
        # First, clear the occluded flag if it's set, as I couldn't get a straight answer about
        # what it actually means (and it clearly doesn't mean "occluded")
        for l in labelled_markers:
            l._params &= ~LabelledMarker._OCCLUDED

        # Fill in missing markers
        markers = set((l.model_id, l.marker_id) for l in labelled_markers)
        missing_markers = self._expected_markers - markers
        if missing_markers:
            for model_id, marker_id in missing_markers:
                # Get model-solved position from markerset
                try:
                    model_name = self._model_names[model_id]
                    markerset_candidates = [m for m in markersets if m.name == model_name]
                    assert len(markerset_candidates) == 1
                    markerset = markerset_candidates[0]
                    position = markerset.markers[marker_id - 1]
                except (KeyError, AssertionError):
                    self._log.warning('Tried to recreate occluded marker %i for unknown model %i',
                                      marker_id, model_id)
                    print(self._model_names)
                    continue
                # Construct labelled marker
                params = LabelledMarker._OCCLUDED | LabelledMarker._MODEL_SOLVED | \
                    LabelledMarker._HAS_MODEL
                reconstructed_marker = LabelledMarker(
                    model_id=model_id, marker_id=marker_id, position=position,
                    size=0.1,  # Arbitrary small size
                    params=params,
                    residual=1  # Arbitrary large residual
                )
                labelled_markers.append(reconstructed_marker)
            labelled_markers.sort(key=lambda lm: (lm.model_id, lm.marker_id))

    def _handle_frame(self, frame_message, received_time):
        rigid_bodies = frame_message.rigid_bodies
        skeletons = frame_message.skeletons
        labelled_markers = frame_message.labelled_markers
        markersets = frame_message.markersets

        if labelled_markers and markersets:
            # Labelled markers and "solver replaces occlusion" are both on, so we can fill in the
            # missing labelled markers from the corresponding markerset
            self._do_occlusion_workaround(labelled_markers, markersets)

        timestamp_and_latency = TimestampAndLatency._calculate(
            received_time, frame_message.timing_info, self._clock_synchronizer)
        self._callback(rigid_bodies, skeletons, labelled_markers, timestamp_and_latency)

        if frame_message.tracked_models_changed:
            self._log.info('Tracked models have changed, requesting new model definitions')
            self._conn.send_packet(protocol.serialize(
                protocol.RequestModelDefinitionsMessage()))

    def _handle_model_definitions(self, model_definitions_message):
        """Update local list of rigid body id:name mappings.

        :type model_definitions_message: protocol.ModelDefinitionsMessage
        """
        self._model_definitions = model_definitions_message.models

        rigid_bodies = [m for m in self._model_definitions if type(m) is RigidBodyDescription]
        self._expected_markers = set((r.id_, i + 1) for r in rigid_bodies
                                     for i in range(len(r.marker_positions)))
        self._model_names = {}
        for m in self._model_definitions:
            try:
                self._model_names[m.id_] = m.name
            except AttributeError:
                pass

        # TODO: Figure out what to do when there are duplicate streaming IDs

        # Sanity check
        rigid_body_ids = set(m.id_ for m in rigid_bodies)
        if len(rigid_body_ids) != len(rigid_bodies):
            names = collections.defaultdict(list)
            for b in rigid_bodies:
                names[b.id_].append(b.name)
            duplicates = {id_: names_ for id_, names_ in names.items() if len(names_) > 1}
            self._log.warning('Warning: multiple rigid bodies with the same streaming ID detected ({})'
                              .format(duplicates))

        self._call_model_callback()

    def run_once(self, timeout=None):
        """Receive and process one message."""
        message_id, payload, received_time = self._conn.wait_for_packet(timeout)
        if message_id is None:
            self._log.warning('Timed out waiting for packet')
            return
        if message_id == protocol.MessageId.FrameOfData:
            if self._callback:
                frame_message = protocol.deserialize_payload(message_id, payload)
                self._handle_frame(frame_message, received_time)
        elif message_id == protocol.MessageId.ModelDef:
            model_definitions_message = protocol.deserialize_payload(message_id, payload)
            self._handle_model_definitions(model_definitions_message)
        elif message_id == protocol.MessageId.EchoResponse:
            echo_response_message = protocol.deserialize_payload(message_id, payload)
            self._clock_synchronizer.handle_echo_response(echo_response_message, received_time)
        else:
            self._log.error('Unhandled message type:', message_id.name)
        self._clock_synchronizer.update(self._conn)

    def spin(self, timeout=None):
        """Continuously receive and process messages."""
        try:
            while True:
                self.run_once(timeout)
        except (KeyboardInterrupt, SystemExit):
            self._log.info('Exiting')
