"""Tests for Client class."""

import mock
import pytest

import natnet
from natnet.fakes import FakeClockSynchronizer, FakeConnection
from natnet.protocol.ModelDefinitionsMessage import ModelDefinitionsMessage, RigidBodyDescription


@pytest.fixture(scope='module', autouse=True)
def patch_socket_module():
    """Make sure nothing creates a socket by mistake."""

    def throw(*args, **kwargs):
        raise RuntimeError('Tried to create socket during testing')

    import socket
    old_socket = socket.socket
    socket.socket = throw
    yield
    socket.socket = old_socket


def test_patching_socket_module_worked():
    import socket

    with pytest.raises(RuntimeError):
        s = socket.socket()  # noqa: F841


@pytest.fixture(scope='module')
def test_packets():
    server_info_packet = open('test_data/serverinfo_packet_v3.bin', 'rb').read()
    mocapframe_packet = open('test_data/mocapframe_packet_v3.bin', 'rb').read()
    modeldef_packet = open('test_data/modeldef_packet_v3.bin', 'rb').read()
    return server_info_packet, mocapframe_packet, modeldef_packet


@pytest.fixture(scope='module')
def test_messages(test_packets):
    server_info_packet, mocapframe_packet, modeldef_packet = test_packets
    server_info_message = natnet.protocol.deserialize(server_info_packet)  # type: natnet.protocol.ServerInfoMessage
    mocapframe_message = natnet.protocol.deserialize(mocapframe_packet)  # type: natnet.protocol.MocapFrameMessage
    modeldef_message = natnet.protocol.deserialize(modeldef_packet)   # type: natnet.protocol.ModelDefinitionsMessage
    return server_info_message, mocapframe_message, modeldef_message


@pytest.fixture
def client_with_fakes(test_messages):
    """Create client with fake connection and clock synchronizer."""
    server_info_message, _, _ = test_messages
    conn = FakeConnection()
    log = natnet.Logger()
    clock_synchronizer = FakeClockSynchronizer(server_info_message, log)
    return natnet.Client(conn, clock_synchronizer, log)


def test_client_calls_callback_for_mocapframe(client_with_fakes, test_packets, test_messages):
    client = client_with_fakes
    client._conn.rate = 1000  # Enable timestamping
    _, mocapframe_packet, _ = test_packets
    _, mocapframe_message, _ = test_messages

    # Fake current time to be the time the server sent the message
    received_time = client._clock_synchronizer.server_ticks_to_seconds(
            mocapframe_message.timing_info.transmit_timestamp)
    with mock.patch('natnet.comms.timeit.default_timer', lambda: received_time):
        # Set wait_for_message to return a FrameOfData the first time, then raise SystemExit the
        # second time
        client._conn.add_packet(mocapframe_packet)

        # Run the Client main loop with an inspectable callback
        callback = mock.Mock()
        client.set_callback(callback)
        client.spin()

    # Check call arguments
    callback.assert_called_once()
    (rigid_bodies, skeletons, labelled_markers, timing), _ = callback.call_args
    assert rigid_bodies == mocapframe_message.rigid_bodies
    assert skeletons == mocapframe_message.skeletons
    assert labelled_markers == mocapframe_message.labelled_markers
    assert timing.timestamp == pytest.approx(427584.91)
    # SampleClient says 5.5ms
    assert timing.system_latency == pytest.approx(0.005495071)
    # From fake time
    assert timing.transit_latency == 0
    assert timing.processing_latency == 0
    assert timing.latency == pytest.approx(0.005495071)


def test_client_fills_in_occluded_markers(client_with_fakes):
    client = client_with_fakes

    # Oops, forgot to save model definitions when I captured this frame, better make something up
    body_def = RigidBodyDescription(
        name='FakeTree', id_=7, parent_id=-1, offset_from_parent=(0.0, 0.0, 0.0),
        marker_positions=[(0, 0, 0)]*5, required_active_labels=[0, 0, 0, 0, 0])
    modeldef_message = ModelDefinitionsMessage([body_def])
    client._conn.add_message(modeldef_message)

    mocapframe_packet_occluded = open('test_data/mocapframe_packet_occluded_v3.bin', 'rb').read()
    client._conn.add_packet(mocapframe_packet_occluded)

    callback = mock.Mock()
    client.set_callback(callback)
    client.spin()

    callback.assert_called_once()
    (rigid_bodies, skeletons, labelled_markers, timing), _ = callback.call_args

    # All labelled markers should be present
    assert len(labelled_markers) == 5

    # Labelled markers should be in order
    assert labelled_markers[0].marker_id == 1
    assert labelled_markers[1].marker_id == 2
    assert labelled_markers[2].marker_id == 3
    assert labelled_markers[3].marker_id == 4
    assert labelled_markers[4].marker_id == 5

    # Only actually-occluded markers should have occluded set
    assert labelled_markers[0].occluded
    assert not labelled_markers[1].occluded
    assert not labelled_markers[2].occluded
    assert not labelled_markers[3].occluded
    assert not labelled_markers[4].occluded

    # All labelled markers should have the same position as the corresponding markerset marker
    assert labelled_markers[0].position == (-0.1684834212064743, 0.2831866443157196, 0.4908055067062378)
    assert labelled_markers[1].position == (-0.13990189135074615, 0.4736242890357971, 0.4416866898536682)
    assert labelled_markers[2].position == (-0.12027807533740997, 0.18182460963726044, 0.462637722492218)
    assert labelled_markers[3].position == (-0.24141907691955566, 0.2144920825958252, 0.4675152003765106)
    assert labelled_markers[4].position == (-0.10057533532381058, 0.26159632205963135, 0.49067628383636475)


def test_client_calls_synchronizer_for_echo_response(client_with_fakes):
    client = client_with_fakes
    echo_response_message = natnet.protocol.EchoResponseMessage(0, 0)

    # Set wait_for_message to return a FrameOfData the first time, then raise SystemExit the second time
    client._conn.add_message(echo_response_message)

    # Run the Client main loop
    client._clock_synchronizer.handle_echo_response = mock.Mock()
    client.spin()

    # Check call arguments
    client._clock_synchronizer.handle_echo_response.assert_called_once()
    args, kwargs = client._clock_synchronizer.handle_echo_response.call_args
    assert args == (echo_response_message, 0)


def test_client_calls_callback_for_modeldef(client_with_fakes, test_packets, test_messages):
    client = client_with_fakes
    _, _, modeldef_packet = test_packets
    _, _, modeldef_message = test_messages

    client._conn.add_packet(modeldef_packet)
    callback = mock.Mock()
    client.set_model_callback(callback)

    # Check callback called immediately
    callback.assert_called_once()
    # Fake client doesn't have any model definitions on startup
    assert callback.call_args_list[0] == mock.call([], [], [])

    # Check callback called with new model definitions when ModelDef message is received
    client.run_once()
    assert callback.call_count == 2
    args, kwargs = callback.call_args_list[1]
    assert kwargs == {}
    rigid_body_descriptions, skeleton_descriptions, markerset_descriptions = args
    assert len(rigid_body_descriptions) == 1
    assert rigid_body_descriptions[0].name == 'RaceQuad'
    assert skeleton_descriptions == []
    assert len(markerset_descriptions) == 2
    assert markerset_descriptions[0].name == 'RaceQuad'
    assert markerset_descriptions[1].name == 'all'


def test_client_connect(test_packets):
    with mock.patch('natnet.comms.ClockSynchronizer'):
        with mock.patch('natnet.comms.Connection') as MockedConnectionCls:
            # Make Connection.open(*) return a fake connection
            server_info_packet, _, modeldef_packet = test_packets
            conn = FakeConnection([server_info_packet, modeldef_packet])
            mock_conn = mock.Mock(wraps=conn)  # Add assert_called etc
            MockedConnectionCls.open.return_value = mock_conn

            # Call Client.connect
            client = natnet.Client.connect('192.168.0.106')

    # Checks
    assert client._conn.send_message.call_args_list[0] == mock.call(natnet.protocol.ConnectMessage())
    assert client._conn.send_message.call_args_list[1] == mock.call(natnet.protocol.RequestModelDefinitionsMessage())
    assert conn.packets_remaining == 0
    client._conn.bind_data_socket.assert_called_once()
