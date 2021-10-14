"""Tests for fakes."""

import timeit

import mock
import pytest

from natnet.fakes import FakeConnection, SingleFrameFakeClient


def test_fakeconnection_repeat():
    """Test fakes.FakeConnection repeats when repeat=True."""
    server_info_packet = open('test_data/serverinfo_packet_v3.bin', 'rb').read()
    conn = FakeConnection([server_info_packet], repeat=True)

    assert conn.wait_for_packet_raw() == (server_info_packet, 0)
    assert conn.wait_for_packet_raw() == (server_info_packet, 0)
    assert conn.wait_for_packet_raw() == (server_info_packet, 0)
    assert conn.wait_for_packet_raw() == (server_info_packet, 0)


def test_single_frame_fake_client_repeats_frame():
    """Test public FakeClient interfaced in demo script and ROS node."""
    client = SingleFrameFakeClient.fake_connect()

    # Check callback is called
    callback = mock.Mock()
    client.set_callback(callback)
    client.run_once()
    callback.assert_called_once()
    (rigid_bodies, skeletons, labelled_markers, timing), _ = callback.call_args
    # Don't really care what these are
    assert rigid_bodies is not None
    assert labelled_markers is not None
    assert timing is not None

    # Try again to make sure it repeats
    callback = mock.Mock()
    client.set_callback(callback)
    client.run_once()
    callback.assert_called_once()


def test_single_frame_fake_client_has_modeldef():
    client = SingleFrameFakeClient.fake_connect()

    # Check callback is called
    callback = mock.Mock()
    client.set_model_callback(callback)
    callback.assert_called_once()
    (rigid_bodies, skeletons, markersets), _ = callback.call_args
    # Don't really care what these are
    assert len(rigid_bodies) == 1
    assert len(skeletons) == 0
    assert len(markersets) == 2


def test_single_frame_fake_client_with_rate():
    """Test setting rate parameter performs rate limiting."""
    rate = 50
    client = SingleFrameFakeClient.fake_connect(rate=rate)

    times = []
    client.set_callback(lambda r, s, l, t: times.append(timeit.default_timer()))
    for i in range(10):
        client.run_once()

    for i in range(1, len(times)):
        assert times[i] - times[i - 1] == pytest.approx(1.0/rate, abs=1e3)


def test_single_frame_fake_client_timestamps():
    """Test setting rate parameter produces sensible fake timestamps and transit times."""
    rate = 50
    client = SingleFrameFakeClient.fake_connect(rate=rate)

    def check_time(r, s, l, t):
        assert t.timestamp == pytest.approx(timeit.default_timer(), abs=1e3)
        assert t.transit_latency == pytest.approx(0, abs=1e3)

    client.set_callback(check_time)
    for i in range(10):
        client.run_once()
